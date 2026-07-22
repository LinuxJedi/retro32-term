/*
 * Retro32 Terminal - a tiny serial ANSI terminal for the Retro32 BBS disk.
 *
 * Bridges the Amiga serial port to console.device on its own 8-colour
 * hires screen, so whatever is on the other end of the serial line (the
 * Copperline browser bridge, --serial-connect, a null-modem cable) fills
 * the screen and the keyboard talks back. The Amiga console device does
 * the ANSI work: CSI colours (SGR 30-37/40-47), cursor motion and erase
 * sequences render natively in Topaz 8, which is the font Amiga-oriented
 * BBSes such as Retro32 design for.
 *
 * The serial port is driven at the hardware level (Paula SERPER/SERDAT/
 * SERDATR plus an RBF interrupt handler installed with SetIntVector),
 * not through serial.device: Kickstart 2.0+ keeps serial.device on the
 * Workbench disk (DEVS:), and the AROS ROM Copperline bundles has no
 * openable serial.device from a bare boot floppy either - a self
 * contained boot disk cannot rely on it. Driving Paula directly works
 * identically on every Kickstart and on AROS. This disk is a dedicated
 * kiosk: it assumes nothing else is using the serial hardware.
 *
 * Runs on Kickstart 1.3 as well as 2.0+ and the AROS ROM Copperline
 * bundles. The three 2.0-only dependencies are fenced off: build.sh
 * links libnix13 (stock libnix backs the compiler's 32-bit divide and
 * multiply helpers with utility.library, which 1.3 does not have), the
 * message ports and IORequests are hand-rolled statics (CreateMsgPort/
 * CreateIORequest are V36+), and the screen open falls back from
 * OpenScreenTagList to OpenScreen plus LoadRGB4/ShowTitle on a V34
 * intuition. Public domain (see LICENSE).
 *
 * Build: see build.sh (m68k-amigaos-gcc / bebbo toolchain).
 */

/* Self-contained SDK slice (see the header for why): no NDK headers are
 * required beyond the toolchain's own <inline/macros.h>. */
#include "amiga-mini.h"

/* The line settings: 8N1, 19200 everywhere. Paula buffers a single
 * received byte, so every byte must be serviced within one character time.
 * Kickstart's exec dispatches the RBF vector with the flag still set and
 * comfortably services 19200 on a stock 68000 (with the BLTPRI note in
 * setup). AROS's scheduler masks all interrupts in stretches of roughly a
 * millisecond -- longer than a 19200 character time (~520 us) -- which
 * used to force 4800 here; the terminal now sidesteps the scheduler
 * entirely on AROS by running a permanent-Forbid() kiosk loop (see
 * kiosk_loop below), which lets AROS hold 19200 too. The defines stay
 * separate as tuning knobs. The Copperline bridge itself is not paced;
 * this only sets how fast the guest side reads and writes. */
#define BAUD_KICKSTART 19200UL
#define BAUD_AROS 19200UL

/* SERPER divisor from the PAL colour clock (an NTSC machine lands within
 * 1%, well inside the UART's tolerance at these rates). */
#define PAL_COLOR_CLOCK 3546895UL
#define SERPER_FOR(baud) ((UWORD)((PAL_COLOR_CLOCK + (baud) / 2) / (baud) - 1))

static ULONG baud;
static BOOL on_aros;

struct Library *ConsoleDevice;

/* How much serial data to move to the console per wakeup. */
#define CHUNK 4096

/* Received-byte ring between the RBF interrupt handler and the main loop.
 * Power of two, and it must divide 65536: the indices are UWORDs because
 * a 68000 reads/writes a word atomically but a longword as two accesses,
 * and each side reads the other side's index -- a torn 32-bit counter
 * silently drops or repeats bytes. The interrupt writes rx_head, the main
 * loop rx_tail; wrap-around arithmetic stays correct in 16 bits.
 *
 * Sized to absorb a whole BBS page, not just a burst: the AROS console
 * renders far slower than the 19200 line rate (a few hundred bytes per
 * second on SGR-heavy art), so the ring backlogs most of a page while
 * the console catches up. 32 KiB is ~17 seconds of line-rate data,
 * comfortably above any screenful a BBS sends; the 8 KiB it replaced
 * overflowed mid-page on the larger Retro32 menus. */
#define RING 32768
static volatile UBYTE ring[RING];
static volatile UWORD rx_head, rx_tail;

/* Set by the interrupt handler when receive data was provably lost (a
 * Paula overrun, or a byte arriving with the ring full); the main loop
 * turns it into a visible marker rather than silently corrupting the
 * session. */
static volatile UBYTE rx_lost;

struct Library *IntuitionBase;
struct Library *GfxBase;

/* libnix provides stdio over the shell console; declared by hand because
 * this build deliberately uses no libc headers. */
extern int printf(const char *fmt, ...);

/* ANSI palette in SGR order (30..37): black, red, green, yellow, blue,
 * magenta, cyan, white - so console pen N is ANSI colour N. Values are
 * 12-bit 0x0RGB, one nibble per gun. */
static const UWORD ansi_rgb4[8] = {
    0x0000, 0x0C22, 0x02C2, 0x0CC2, 0x035E, 0x0C3C, 0x03CC, 0x0EEE,
};

static struct Screen *screen;
static struct Window *window;

static struct MsgPort *con_rd_port, *con_wr_port;
static struct IOStdReq *con_rd, *con_wr;
static BOOL con_open;

static struct Task *term_task;
static BYTE ser_sigbit = -1;
static ULONG ser_sigmask;
static struct Interrupt rbf_interrupt;
static struct Interrupt *old_rbf;
static BOOL rbf_installed;
static BOOL dtr_asserted;

/* Blank pointer sprite: control words + one bitplane row + terminator,
 * all zero. Must live in chip RAM and stay allocated while set. */
#define BLANK_POINTER_BYTES 12
static UWORD *blank_pointer;

static UBYTE con_ch; /* 1-byte console read landing zone */
static UBYTE chunk[CHUNK];

/* --- Paula RBF interrupt ------------------------------------------------- */

/* Exec calls an interrupt handler with d0/d1/a0/a1/a5/a6 free and returns
 * with rts. A normal C function preserves everything else per the ABI, so
 * a bare jsr/rts stub is a valid handler body. The asm-label declaration
 * pins the symbol name the stub references. */
void rbf_c(void) __asm__("rbf_c");

__asm__("    .text\n"
        "    .globl rbf_stub\n"
        "rbf_stub:\n"
        "    jsr rbf_c\n"
        "    rts\n");
extern void rbf_stub(void) __asm__("rbf_stub");

static void rbf_bank(UWORD datr)
{
    UWORD next = rx_head + 1;
    if ((UWORD)(next - rx_tail) <= RING) {
        ring[rx_head % RING] = (UBYTE)datr;
        rx_head = next;
    } else {
        /* Full: drop this byte and say so. The head must NOT advance
         * past an unwritten slot -- doing so makes the drain loop read
         * whatever the slot held a lap ago, substituting stale bytes
         * into the stream instead of just losing new ones. */
        rx_lost = 1;
    }
    /* OVRUN means a word completed while the previous one was still
     * unserviced: Paula dropped it. Latched until the RBF ack below. */
    if (datr & SERDATR_OVRUN)
        rx_lost = 1;
}

void rbf_c(void)
{
    UWORD datr;

    /* Kickstart's exec enters this handler with RBF still set; AROS's
     * level-5 dispatcher acks INTREQ BEFORE calling handlers, leaving the
     * word latched in SERDATR with the flag already clear. Either way the
     * handler only runs because a word arrived, so take the first byte
     * unconditionally, ack, then drain any further words that complete
     * while we are here (the flag reasserts per word). */
    rbf_bank(CUSTOM_SERDATR);
    CUSTOM_INTREQ = INTF_RBF;
    while ((datr = CUSTOM_SERDATR) & SERDATR_RBF) {
        rbf_bank(datr);
        CUSTOM_INTREQ = INTF_RBF;
    }
    Signal(term_task, ser_sigmask);
}

/* Transmit one byte: wait for the transmit buffer, then write data plus
 * the 8N1 stop bit. The bound only trips if the UART wedges; a byte time
 * at BAUD is a few hundred microseconds. */
static void ser_putc(UBYTE c)
{
    ULONG guard = 2000000;
    while (!(CUSTOM_SERDATR & SERDATR_TBE) && --guard)
        ;
    CUSTOM_SERDAT = 0x0100 | c;
}

static void ser_write(const UBYTE *data, LONG len)
{
    LONG i;
    for (i = 0; i < len; i++)
        ser_putc(data[i]);
}

/* --- console ------------------------------------------------------------- */

static void con_write(const UBYTE *data, LONG len)
{
    con_wr->io_Command = CMD_WRITE;
    con_wr->io_Data = (APTR)data;
    con_wr->io_Length = len;
    DoIO((struct IORequest *)con_wr);
}

static void con_puts(const char *s)
{
    LONG n = 0;
    while (s[n])
        n++;
    con_write((const UBYTE *)s, n);
}

static void con_put_num(ULONG v)
{
    char buf[11];
    WORD i = 10;
    buf[i] = 0;
    if (!v)
        buf[--i] = '0';
    while (v) {
        buf[--i] = (char)('0' + v % 10);
        v /= 10;
    }
    con_puts(&buf[i]);
}

static void start_con_read(void)
{
    con_rd->io_Command = CMD_READ;
    con_rd->io_Data = &con_ch;
    con_rd->io_Length = 1;
    SendIO((struct IORequest *)con_rd);
}

/* Debug aid: dump incoming bytes as hex instead of passing them through. */
#define DEBUG_HEX 0

/* --- ANSI-to-console shim -------------------------------------------------
 * Incoming CSI sequences are buffered whole (they arrive split across
 * interrupt-sized chunks, and a console parser fed a partial sequence can
 * mis-render it) and re-emitted in the 8-bit CSI form. Multi-parameter SGR
 * sequences (ESC[1;33m) are split into consecutive single-parameter ones:
 * the AROS console only understands the single-parameter form, and the
 * split is semantically identical on Kickstart. Parameterized erase
 * sequences (ESC[2J page clears, ESC[2K line clears) are translated to
 * console equivalents (see emit_csi_erase), and HVP (ESC[y;xf) to CUP. */
#define SEQ_MAX 48
static UBYTE seq[SEQ_MAX];
static WORD seq_len;
/* 0 = plain data, 1 = ESC seen (awaiting '['), 2 = collecting CSI body,
 * 3 = discarding an overlong sequence */
static WORD seq_state;

static LONG emit(LONG n, UBYTE b)
{
    if (n < CHUNK)
        chunk[n++] = b;
    return n;
}

static LONG emit_csi_split_sgr(LONG n);

static WORD csi_first_value(void)
{
    WORD v = 0;
    WORD i;
    for (i = 0; i < seq_len && seq[i] >= '0' && seq[i] <= '9'; i++)
        v = v * 10 + (seq[i] - '0');
    return v;
}

/* Re-emit the sequence keeping at most maxp parameters: the consoles
 * take one parameter for most commands (two for H), and the AROS
 * console rejects a sequence outright when it carries more. */
static LONG emit_csi_params(LONG n, UBYTE final, WORD maxp)
{
    WORD i, p = 1;
    n = emit(n, 0x9B);
    for (i = 0; i < seq_len; i++) {
        if (seq[i] == ';' && ++p > maxp)
            break;
        n = emit(n, seq[i]);
    }
    return emit(n, final);
}

/* ED/EL (CSI Pn J / CSI Pn K): the Amiga console's J and K take no
 * parameter and only erase toward the end of the display/line, while
 * BBSes lean on the ANSI.SYS forms - ESC[2J clears the whole screen and
 * homes the cursor, ESC[2K blanks the whole line and returns to column
 * 1. Translate: 2J (and xterm's 3J) becomes a formfeed, which is the
 * console's own whole-screen clear-and-home on Kickstart and AROS
 * alike; 2K becomes CR plus erase-to-end; 0 or no parameter passes
 * through with the parameter stripped; the erase-backwards forms (1J,
 * 1K) have no console equivalent and are rare in BBS output, so they
 * are dropped. */
static LONG emit_csi_erase(LONG n, UBYTE final)
{
    WORD v = csi_first_value();
    if (v >= 2) {
        if (final == 'J')
            return emit(n, 0x0C);
        n = emit(n, '\r');
    } else if (v == 1) {
        return n;
    }
    n = emit(n, 0x9B);
    return emit(n, final);
}

/* CHA (CSI Pn G, cursor to absolute column n): no console equivalent,
 * but CR plus cursor-forward lands on the same cell. */
static LONG emit_csi_cha(LONG n)
{
    WORD col = csi_first_value();
    n = emit(n, '\r');
    if (col > 1) {
        UBYTE d[5];
        WORD v = col - 1, k = 0;
        while (v) {
            d[k++] = (UBYTE)('0' + v % 10);
            v /= 10;
        }
        n = emit(n, 0x9B);
        while (k)
            n = emit(n, d[--k]);
        n = emit(n, 'C');
    }
    return n;
}

/* A complete CSI sequence: forward it only in a form the console is
 * known to parse. This is a whitelist because the failure mode for
 * anything else is not "ignored": the AROS console's command scanner
 * runs PAST the final byte of a sequence it cannot match, consuming
 * the output that follows and printing fragments of it as text (seen
 * live as mangled menu lines when the BBS sent its ECMA-48 font
 * selection, ESC[0;40 D). A real terminal parses everything and
 * silently ignores what it does not implement, so unknown sequences
 * are swallowed whole here. */
static LONG emit_csi(LONG n, UBYTE final)
{
    WORD i;

    /* Private markers, intermediates, and colon subparameters
     * (ESC[?25l, ESC[0;40 D, ESC[38:5:201m) have no console form. */
    for (i = 0; i < seq_len; i++) {
        if (!((seq[i] >= '0' && seq[i] <= '9') || seq[i] == ';'))
            return n;
    }

    switch (final) {
    case 'm':
        for (i = 0; i < seq_len; i++) {
            if (seq[i] == ';')
                return emit_csi_split_sgr(n);
        }
        return emit_csi_params(n, 'm', 1);
    case 'J':
    case 'K':
        return emit_csi_erase(n, final);
    case 'f': /* HVP: same motion as CUP, which the console has */
    case 'H':
        return emit_csi_params(n, 'H', 2);
    case 'G':
        return emit_csi_cha(n);
    case 'A':
    case 'B':
    case 'C':
    case 'D':
    case 'E':
    case 'F':
    case 'S':
    case 'T':
    case '@':
    case 'L':
    case 'M':
    case 'P':
        return emit_csi_params(n, final, 1);
    case 'n':
        /* DSR: the Kickstart console answers it on the read stream
         * (which the main loop forwards up the line); the AROS
         * console rejects it. */
        if (!on_aros)
            return emit_csi_params(n, 'n', 1);
        return n;
    default:
        return n;
    }
}

static LONG emit_csi_split_sgr(LONG n)
{
    /* seq[] holds the CSI body without introducer or final byte. */
    WORD i = 0;
    while (i <= seq_len) {
        n = emit(n, 0x9B);
        if (i == seq_len || seq[i] == ';') {
            n = emit(n, '0'); /* empty parameter means reset */
        } else {
            while (i < seq_len && seq[i] != ';')
                n = emit(n, seq[i++]);
        }
        n = emit(n, 'm');
        i++; /* skip the ';' (or step past the end) */
    }
    return n;
}

static LONG emit_byte(LONG n, UBYTE b)
{
    switch (seq_state) {
    case 0:
        if (b == 0x1B) {
            seq_state = 1;
        } else if (b == 0x9B) {
            seq_state = 2;
            seq_len = 0;
        } else {
            n = emit(n, b);
        }
        return n;
    case 1:
        if (b == '[') {
            seq_state = 2;
            seq_len = 0;
        } else {
            /* Not a CSI: pass the ESC and this byte through untouched
             * (the console understands other ESC sequences directly). */
            seq_state = 0;
            n = emit(n, 0x1B);
            n = emit(n, b);
        }
        return n;
    case 3:
        /* Discarding an overlong sequence: swallow until its final. */
        if (b >= 0x40 && b <= 0x7E)
            seq_state = 0;
        return n;
    default:
        break;
    }

    if (b >= 0x40 && b <= 0x7E) {
        seq_state = 0;
        return emit_csi(n, b);
    }
    if (seq_len < SEQ_MAX) {
        seq[seq_len++] = b;
    } else {
        /* Longer than any sequence the console knows; drop it rather
         * than spilling a garbage prefix onto the screen. */
        seq_state = 3;
    }
    return n;
}

/* Move everything the interrupt handler has banked into the console in
 * CHUNK-sized writes. */
static void drain_serial(void)
{
    if (rx_lost) {
        rx_lost = 0;
        con_puts("\x9B" "41m\x9B" "37m[LOST]\x9B" "0m");
    }
    while (rx_tail != rx_head) {
        LONG n = 0;
        /* Splitting can expand a byte severalfold; leave headroom. */
        while (rx_tail != rx_head && n < CHUNK - (SEQ_MAX * 4)) {
#if DEBUG_HEX
            static const char hexd[] = "0123456789abcdef";
            UBYTE b = ring[rx_tail % RING];
            chunk[n++] = hexd[b >> 4];
            chunk[n++] = hexd[b & 15];
            chunk[n++] = ' ';
#else
            n = emit_byte(n, ring[rx_tail % RING]);
#endif
            rx_tail++;
        }
        con_write(chunk, n);
    }
}

/* Keystroke from the console: forward it down the line. The console
 * emits 8-bit CSI (0x9B) for special keys; BBSes expect the 7-bit ESC [
 * form, so rewrite that one byte and pass everything else verbatim. */
static void forward_key(UBYTE c)
{
    static const UBYTE esc_bracket[2] = { 0x1B, '[' };
    if (c == 0x9B)
        ser_write(esc_bracket, 2);
    else
        ser_write(&c, 1);
}

/* --- AROS Forbid() kiosk mode ---------------------------------------------
 * AROS's m68k interrupt-exit path (arch/m68k-all/kernel/kernel_intr.c,
 * core_ExitIntr) invokes the task scheduler only while task switching is
 * enabled (TDNestCnt < 0), and that scheduler runs with interrupts
 * masked (IPL 7 / INTENA off) in stretches of roughly a millisecond --
 * longer than a 19200 character time, so Paula's one-byte receive latch
 * gets overwritten and bytes vanish. This disk owns the whole machine,
 * so on AROS the main loop holds Forbid() permanently and never calls
 * Wait() (Wait breaks Forbid while the task sleeps): with TDNestCnt >= 0
 * the dispatch windows never open and the RBF interrupt stays timely.
 * What remains is the level-6 CIA handlers, which Kickstart also has and
 * which fit inside a character time.
 *
 * The price is that no other task ever runs again:
 * - Console writes still work: AROS console.device completes CMD_WRITE
 *   in the caller's context (rom/devs/console/console.c BeginIO leaves
 *   it done_quick), so DoIO never sleeps.
 * - Console CMD_READ would never complete (it is handed to the console
 *   device's task), so the keyboard is read from the CIA-A hardware
 *   here instead: INTENA's PORTS bit is cleared so the OS keyboard
 *   handler no longer swallows the scancodes, then the loop polls the
 *   SP flag, takes the scancode from SDR, performs the KDAT handshake,
 *   and feeds the code through console.device's RawKeyConvert() -- on
 *   AROS a synchronous keymap.library call, safe under Forbid. Key
 *   repeat was input.device's job, so kiosk mode has none. */

static UWORD kbd_qual; /* live IEQUALIFIER_* bits, tracked from raw ups */

/* Delay by watching the vertical beam counter advance: each raster line
 * is 63.5 us, so `lines` transitions bound the wait below from
 * (lines - 1) * 63.5 us without touching any timer hardware. */
static void beam_wait_lines(WORD lines)
{
    UBYTE v = (UBYTE)(CUSTOM_VHPOSR >> 8);
    while (lines > 0) {
        UBYTE w = (UBYTE)(CUSTOM_VHPOSR >> 8);
        if (w != v) {
            v = w;
            lines--;
        }
    }
}

static void kbd_rawkey(UBYTE code)
{
    struct InputEvent ie;
    UBYTE buf[16];
    LONG n, i;

    if ((code & 0x7F) >= 0x60 && (code & 0x7F) <= 0x67) {
        /* The qualifier keys 0x60-0x67 map one-to-one onto the low
         * IEQUALIFIER bits. Caps lock reports down on engage and up on
         * release, so plain down/up tracking is right for it too. */
        UWORD bit = 1 << ((code & 0x7F) - 0x60);
        if (code & IECODE_UP_PREFIX)
            kbd_qual &= ~bit;
        else
            kbd_qual |= bit;
        return;
    }
    /* Ignoring all other ups also swallows the keyboard MCU status
     * codes (F9-FE decode as ups of codes >= 0x79). */
    if (code & IECODE_UP_PREFIX)
        return;

    ie.ie_NextEvent = NULL;
    ie.ie_Class = IECLASS_RAWKEY;
    ie.ie_SubClass = 0;
    ie.ie_Code = code;
    ie.ie_Qualifier = kbd_qual;
    ie.ie_EventAddress = 0;
    ie.ie_Seconds = 0;
    ie.ie_Micros = 0;
    n = RawKeyConvert(&ie, buf, (LONG)sizeof(buf), NULL);
    for (i = 0; i < n; i++)
        forward_key(buf[i]);
}

static void kbd_poll(void)
{
    UBYTE raw;

    if (!(CIAA_ICR & CIAICRF_SP)) /* read clears all CIA-A int flags */
        return;
    raw = CIAA_SDR;

    /* Handshake: drive KDAT low for at least two raster lines (127 us,
     * above the HRM's 85 us minimum) so the keyboard MCU sends the next
     * byte. Ack first, translate after. */
    CIAA_CRA |= CIACRAF_SPMODE;
    beam_wait_lines(3);
    CIAA_CRA &= ~CIACRAF_SPMODE;

    /* SDR holds the byte as shifted off the wire; the classic decode is
     * ror.b then not.b, leaving the rawkey code plus the up bit. */
    kbd_rawkey((UBYTE)~((raw >> 1) | (raw << 7)));
}

/* AROS's exec acks INTREQ RBF BEFORE calling SetIntVector handlers,
 * and that pre-ack is poison at sustained line rate: with the request
 * bit clear, Paula's receive latch is no longer protected, so a word
 * completing between the dispatcher's ack and the handler's SERDATR
 * read OVERWRITES the unread word - silently, because overrun
 * detection only triggers while the request bit is set. The handler
 * cannot close that window from inside the dispatcher, so the kiosk
 * bypasses it: the level-5 autovector (68000 vector 29) is pointed
 * straight at this handler. The request bit then stays set from word
 * completion until the ack below, the latch stays protected the whole
 * way, and the worst any race can do is an honest OVRUN that the
 * [LOST] marker reports. Banking the already-read value after the ack
 * (rather than re-reading) closes the overwrite race on the data
 * itself. No OS calls here: the kiosk main loop busy-polls, so not
 * even Signal() is needed. */
void lvl5_c(void) __asm__("lvl5_c");

__asm__("    .text\n"
        "    .globl lvl5_stub\n"
        "lvl5_stub:\n"
        "    movem.l d0-d1/a0-a1,-(sp)\n"
        "    jsr lvl5_c\n"
        "    movem.l (sp)+,d0-d1/a0-a1\n"
        "    rte\n");
extern void lvl5_stub(void) __asm__("lvl5_stub");

void lvl5_c(void)
{
    UWORD datr;
    while ((datr = CUSTOM_SERDATR) & SERDATR_RBF) {
        CUSTOM_INTREQ = INTF_RBF;
        rbf_bank(datr);
    }
}

#define VEC_LEVEL5 (*(volatile ULONG *)0x74)

/* AROS's m68k WaitBlit (arch/m68k-amiga/graphics/waitblit.S) sets
 * blitter-nasty (DMACON BLTPRI) for the duration of every wait and
 * parks the CPU off the chip bus until the blit completes. With no
 * fast RAM the RBF handler's instruction fetches stall with it, so a
 * console blit longer than a character time (~520 us at 19200; big
 * scrolls and fills run for milliseconds) overruns Paula's one-byte
 * receive latch -- clearing BLTHOG at startup does not help because
 * WaitBlit re-asserts it on every call. The kiosk swaps in a plain
 * BBUSY spin: blits finish a little slower with the CPU sharing the
 * bus, and the receive interrupt stays live. The leading tst.w is
 * the original-Agnus erratum workaround (the first DMACONR read
 * after BltSize can report the blitter idle falsely). */
__asm__("    .text\n"
        "    .globl quiet_waitblit\n"
        "quiet_waitblit:\n"
        "    tst.w 0xdff002\n"
        "1:  btst #6,0xdff002\n"
        "    bne.s 1b\n"
        "    rts\n");
extern void quiet_waitblit(void) __asm__("quiet_waitblit");

#define WAITBLIT_LVO (-228) /* graphics.library LVO 38 */

static void kiosk_loop(void)
{
    Forbid();
    SetFunction(GfxBase, WAITBLIT_LVO, (APTR)quiet_waitblit);
    CUSTOM_INTENA = INTF_PORTS; /* CIA-A ints off the CPU; kbd_poll owns them */

    /* Take the level-5 autovector (see lvl5_c); mask RBF around the
     * swap so the interrupt cannot fire between the two word writes
     * of the 32-bit vector. */
    CUSTOM_INTENA = INTF_RBF;
    VEC_LEVEL5 = (ULONG)lvl5_stub;
    CUSTOM_INTENA = INTF_SETCLR | INTF_RBF;

    for (;;) {
        drain_serial();
        kbd_poll();
    }
}

/* The console renders in the screen's font, and a bare-floppy Kickstart
 * boot can default that to a wider Topaz (observed: 10x9 on 3.1, a
 * 64-column console); BBS pages are drawn for 80 columns, so pin the
 * 8x8 Topaz the layouts assume. */
static struct TextAttr topaz8 = { "topaz.font", 8, 0, 0 };

/* Exec's CreateMsgPort/CreateIORequest are V36+ and this program
 * otherwise runs on Kickstart 1.3, so the ports and requests are static
 * structures initialized by hand. The ports are private (never
 * AddPort'ed), which is all CreateMsgPort produced here anyway; the
 * signal bit is the only real allocation. */
static struct MsgPort port_rd, port_wr;
static struct IOStdReq ioreq_rd, ioreq_wr;

static struct MsgPort *port_init(struct MsgPort *port)
{
    BYTE sig = AllocSignal(-1);
    if (sig < 0)
        return NULL;
    port->mp_Node.ln_Type = NT_MSGPORT;
    port->mp_Flags = PA_SIGNAL;
    port->mp_SigBit = (UBYTE)sig;
    port->mp_SigTask = FindTask(NULL);
    port->mp_MsgList.lh_Head = (struct Node *)&port->mp_MsgList.lh_Tail;
    port->mp_MsgList.lh_Tail = NULL;
    port->mp_MsgList.lh_TailPred = (struct Node *)&port->mp_MsgList.lh_Head;
    return port;
}

static struct IOStdReq *ioreq_init(struct IOStdReq *req, struct MsgPort *port)
{
    req->io_Message.mn_Node.ln_Type = NT_MESSAGE;
    req->io_Message.mn_ReplyPort = port;
    req->io_Message.mn_Length = sizeof(struct IOStdReq);
    return req;
}

static int setup(void)
{
    struct ColorSpec colors[9];
    struct TagItem screen_tags[3];
    struct NewScreen ns;
    struct NewWindow nw;
    WORD height, i;
    BOOL v36;

    /* 33 is Kickstart 1.2's version; every V36-only call below branches
     * on the version actually present. */
    IntuitionBase = OpenLibrary("intuition.library", 33);
    if (!IntuitionBase)
        return 1;
    GfxBase = OpenLibrary("graphics.library", 33);
    if (!GfxBase)
        return 2;
    v36 = IntuitionBase->lib_Version >= 36;

    /* Hand the ANSI palette to intuition itself with SA_Colors: OpenScreen
     * applies it as the last step of its own colour setup, which is the
     * only ordering that reliably beats the OS defaults everywhere. AROS
     * in particular loads its preference palette onto every new screen,
     * and a manual SetRGB4 afterwards did not stick there. */
    for (i = 0; i < 8; i++) {
        colors[i].ColorIndex = i;
        colors[i].Red = (ansi_rgb4[i] >> 8) & 0xF;
        colors[i].Green = (ansi_rgb4[i] >> 4) & 0xF;
        colors[i].Blue = ansi_rgb4[i] & 0xF;
    }
    colors[8].ColorIndex = -1;
    colors[8].Red = colors[8].Green = colors[8].Blue = 0;

    screen_tags[0].ti_Tag = SA_Colors;
    screen_tags[0].ti_Data = (ULONG)colors;
    /* Keep the screen's title bar behind our backdrop window (it would
     * otherwise draw over the top text rows). */
    screen_tags[1].ti_Tag = SA_ShowTitle;
    screen_tags[1].ti_Data = FALSE;
    screen_tags[2].ti_Tag = TAG_DONE;
    screen_tags[2].ti_Data = 0;

    /* The geometry stays in the NewScreen (ViewModes selects hires; a
     * tags-only open picks the default monitor's lores and shows half the
     * columns), with the tags layered on top -- the classic ExtNewScreen
     * pattern -- when intuition is new enough to take tags at all. PAL
     * rows first, NTSC rows if the machine cannot do 256. */
    for (i = 0; i < (WORD)sizeof(ns); i++)
        ((UBYTE *)&ns)[i] = 0;
    ns.Width = 640;
    ns.Depth = 3;
    ns.DetailPen = 0;
    ns.BlockPen = 1;
    ns.ViewModes = HIRES;
    ns.Type = CUSTOMSCREEN | SCREENQUIET;
    ns.Font = (APTR)&topaz8;
    height = 256;
    ns.Height = height;
    screen = v36 ? OpenScreenTagList(&ns, screen_tags) : OpenScreen(&ns);
    if (!screen) {
        height = 200;
        ns.Height = height;
        screen = v36 ? OpenScreenTagList(&ns, screen_tags) : OpenScreen(&ns);
    }
    if (!screen)
        return 3;

    for (i = 0; i < (WORD)sizeof(nw); i++)
        ((UBYTE *)&nw)[i] = 0;
    nw.Width = 640;
    nw.Height = height;
    nw.Flags = WFLG_SIMPLE_REFRESH | WFLG_BACKDROP | WFLG_BORDERLESS
        | WFLG_ACTIVATE | WFLG_RMBTRAP;
    nw.Screen = screen;
    nw.Type = CUSTOMSCREEN;
    window = OpenWindow(&nw);
    if (!window)
        return 4;

    /* A V34 intuition ignored the tags, so do their work by hand: drop
     * the title bar behind the backdrop window and load the palette.
     * The OpenScreen-applies-it ordering SA_Colors exists for only
     * matters on AROS, which is always V36+; on Kickstart a LoadRGB4
     * after the fact sticks. */
    if (!v36) {
        ShowTitle(screen, FALSE);
        LoadRGB4(ViewPortAddress(window), ansi_rgb4, 8);
    }

    /* The kiosk takes no pointer input, so hide the Intuition pointer:
     * a blank 1-row sprite (chip RAM, zeroed) instead of the busy/arrow
     * imagery darting over the text whenever the host mouse moves.
     * Failing to allocate just leaves the normal pointer - cosmetic. */
    blank_pointer = AllocMem(BLANK_POINTER_BYTES, MEMF_CHIP | MEMF_CLEAR);
    if (blank_pointer)
        SetPointer(window, blank_pointer, 1, 16, 0, 0);

    con_rd_port = port_init(&port_rd);
    con_wr_port = port_init(&port_wr);
    if (!con_rd_port || !con_wr_port)
        return 5;
    con_rd = ioreq_init(&ioreq_rd, con_rd_port);
    con_wr = ioreq_init(&ioreq_wr, con_wr_port);

    /* Console attached to our window; clone the opened device into the
     * read request. */
    con_wr->io_Data = window;
    con_wr->io_Length = WINDOW_SIZEOF;
    if (OpenDevice("console.device", CONU_STANDARD,
                   (struct IORequest *)con_wr, 0))
        return 7;
    con_open = TRUE;
    con_rd->io_Device = con_wr->io_Device;
    con_rd->io_Unit = con_wr->io_Unit;
    ConsoleDevice = (struct Library *)con_wr->io_Device;

    /* Take over the serial hardware: our RBF handler banks received bytes
     * and signals the main loop. */
    term_task = FindTask(NULL);
    ser_sigbit = AllocSignal(-1);
    if (ser_sigbit < 0)
        return 8;
    ser_sigmask = 1UL << ser_sigbit;

    rbf_interrupt.is_Node.ln_Type = NT_INTERRUPT;
    rbf_interrupt.is_Node.ln_Name = (char *)"retro32term RBF";
    rbf_interrupt.is_Data = NULL;
    rbf_interrupt.is_Code = rbf_stub;

    /* Console scrolls are big blits; with BLTPRI (blitter-hog) set they
     * lock the CPU off the chip bus for milliseconds at a time, which is
     * longer than a character time and overruns Paula's one-byte receive
     * buffer no matter how fast the RBF handler is. This disk is a
     * dedicated terminal, so trade a slower scroll for a live UART. */
    CUSTOM_DMACON = DMAF_BLITHOG;

    /* AROS exposes a resident no Kickstart has; see the baud note at the
     * top of the file and kiosk_loop. */
    on_aros = FindResident("aros.library") != NULL;
    baud = on_aros ? BAUD_AROS : BAUD_KICKSTART;
    CUSTOM_SERPER = SERPER_FOR(baud);
    old_rbf = SetIntVector(INTB_RBF, &rbf_interrupt);
    rbf_installed = TRUE;
    CUSTOM_INTREQ = INTF_RBF;                       /* clear stale RBF */
    CUSTOM_INTENA = INTF_SETCLR | INTF_INTEN | INTF_RBF;

    /* Raise DTR and RTS the way serial.device's Open does: whatever is on
     * the far end of the wire keys off DTR to know a terminal is ready.
     * The Copperline browser bridge defers its BBS dial until the guest
     * asserts DTR, so raising it only here - after the console and the RBF
     * handler are live - guarantees the BBS greeting lands on a screen
     * that can show it. Bits 7:6 only; PA5-PA0 belong to other lines. */
    CIAB_DDRA |= CIAF_COMDTR | CIAF_COMRTS;
    CIAB_PRA &= (UBYTE)~(CIAF_COMDTR | CIAF_COMRTS);
    dtr_asserted = TRUE;

    return 0;
}

static void cleanup(void)
{
    if (dtr_asserted) /* hang up: drop DTR/RTS so the far end sees us go */
        CIAB_PRA |= CIAF_COMDTR | CIAF_COMRTS;
    if (rbf_installed) {
        CUSTOM_INTENA = INTF_RBF; /* disable RBF, leave the master alone */
        CUSTOM_INTREQ = INTF_RBF;
        SetIntVector(INTB_RBF, old_rbf);
    }
    if (ser_sigbit >= 0)
        FreeSignal(ser_sigbit);
    if (con_rd && con_rd->io_Device && !CheckIO((struct IORequest *)con_rd)) {
        AbortIO((struct IORequest *)con_rd);
        WaitIO((struct IORequest *)con_rd);
    }
    if (con_open)
        CloseDevice((struct IORequest *)con_wr);
    /* The ports and requests are static; only the ports' signal bits
     * were allocated. */
    if (con_wr_port)
        FreeSignal(con_wr_port->mp_SigBit);
    if (con_rd_port)
        FreeSignal(con_rd_port->mp_SigBit);
    if (blank_pointer) {
        if (window)
            ClearPointer(window);
        FreeMem(blank_pointer, BLANK_POINTER_BYTES);
        blank_pointer = NULL;
    }
    if (window)
        CloseWindow(window);
    if (screen)
        CloseScreen(screen);
    if (GfxBase)
        CloseLibrary(GfxBase);
    if (IntuitionBase)
        CloseLibrary(IntuitionBase);
}

int main(void)
{
    ULONG con_sig, sigs;

    {
        int rc = setup();
        if (rc) {
            printf("term: setup failed (step %d)\n", rc);
            cleanup();
            return 20;
        }
    }

    con_puts("\x9B" "37m\x9B" "1mRetro32 Terminal\x9B" "0m\x9B" "37m  ");
    con_put_num(baud);
    /* DTR is up (setup raised it), so a bridge armed with a deferred
     * dial connects right about now and the far end's greeting simply
     * appears. Do not invite a blind Return: at a BBS login prompt an
     * unsolicited Return reads as an empty name and starts the new-user
     * flow. */
    con_puts(" 8N1, ANSI/Topaz\r\n"
             "Serial line ready - click Connect on the page if you have not already.\r\n"
             "\r\n");

    /* On AROS take over the machine for good; see the kiosk_loop note.
     * It never returns (an exit would need Permit and a rate the AROS
     * scheduler can survive; the kiosk has no reason to exit -- the
     * keyboard MCU still honours Ctrl-Amiga-Amiga for a reboot).
     * NO_KIOSK builds keep the old Wait() loop on AROS as a test control
     * for demonstrating the drops the kiosk exists to avoid. */
#ifndef NO_KIOSK
    if (on_aros)
        kiosk_loop();
#endif

    start_con_read();
    con_sig = 1UL << con_rd_port->mp_SigBit;

    for (;;) {
        sigs = Wait(con_sig | ser_sigmask | SIGBREAKF_CTRL_C);
        if (sigs & SIGBREAKF_CTRL_C)
            break;
        drain_serial();
        if (GetMsg(con_rd_port)) {
            forward_key(con_ch);
            start_con_read();
        }
    }

    cleanup();
    return 0;
}
