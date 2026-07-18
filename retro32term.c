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
 * Needs Kickstart 2.0+ (or the AROS ROM Copperline bundles) for
 * CreateMsgPort/CreateIORequest. Public domain (see LICENSE).
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
 * loop rx_tail; wrap-around arithmetic stays correct in 16 bits. */
#define RING 8192
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
    if ((UWORD)(next - rx_tail) <= RING)
        ring[rx_head % RING] = (UBYTE)datr;
    else
        rx_lost = 1;
    rx_head = next;
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
 * split is semantically identical on Kickstart. */
#define SEQ_MAX 48
static UBYTE seq[SEQ_MAX];
static WORD seq_len;
/* 0 = plain data, 1 = ESC seen (awaiting '['), 2 = collecting CSI body */
static WORD seq_state;

static LONG emit(LONG n, UBYTE b)
{
    if (n < CHUNK)
        chunk[n++] = b;
    return n;
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
    WORD i;

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
    default:
        break;
    }

    if (b >= 0x40 && b <= 0x7E) {
        /* Final byte: re-emit the whole sequence. */
        seq_state = 0;
        if (b == 'm') {
            for (i = 0; i < seq_len; i++) {
                if (seq[i] == ';')
                    return emit_csi_split_sgr(n);
            }
        }
        n = emit(n, 0x9B);
        for (i = 0; i < seq_len; i++)
            n = emit(n, seq[i]);
        n = emit(n, b);
    } else if (seq_len < SEQ_MAX) {
        seq[seq_len++] = b;
    } else {
        /* Overlong sequence: flush it raw rather than losing data. */
        seq_state = 0;
        n = emit(n, 0x9B);
        for (i = 0; i < seq_len; i++)
            n = emit(n, seq[i]);
        n = emit(n, b);
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

static void kiosk_loop(void)
{
    Forbid();
    CUSTOM_INTENA = INTF_PORTS; /* CIA-A ints off the CPU; kbd_poll owns them */
    for (;;) {
        drain_serial();
        kbd_poll();
    }
}

static int setup(void)
{
    struct ColorSpec colors[9];
    struct TagItem screen_tags[3];
    struct NewScreen ns;
    struct NewWindow nw;
    WORD height, i;

    IntuitionBase = OpenLibrary("intuition.library", 36);
    if (!IntuitionBase)
        return 1;
    GfxBase = OpenLibrary("graphics.library", 36);
    if (!GfxBase)
        return 2;

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
     * pattern. PAL rows first, NTSC rows if the machine cannot do 256. */
    for (i = 0; i < (WORD)sizeof(ns); i++)
        ((UBYTE *)&ns)[i] = 0;
    ns.Width = 640;
    ns.Depth = 3;
    ns.DetailPen = 0;
    ns.BlockPen = 1;
    ns.ViewModes = HIRES;
    ns.Type = CUSTOMSCREEN | SCREENQUIET;
    height = 256;
    ns.Height = height;
    screen = OpenScreenTagList(&ns, screen_tags);
    if (!screen) {
        height = 200;
        ns.Height = height;
        screen = OpenScreenTagList(&ns, screen_tags);
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

    con_rd_port = CreateMsgPort();
    con_wr_port = CreateMsgPort();
    if (!con_rd_port || !con_wr_port)
        return 5;
    con_rd = (struct IOStdReq *)CreateIORequest(con_rd_port,
                                                sizeof(struct IOStdReq));
    con_wr = (struct IOStdReq *)CreateIORequest(con_wr_port,
                                                sizeof(struct IOStdReq));
    if (!con_rd || !con_wr)
        return 6;

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

    return 0;
}

static void cleanup(void)
{
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
    if (con_wr)
        DeleteIORequest(con_wr);
    if (con_rd)
        DeleteIORequest(con_rd);
    if (con_wr_port)
        DeleteMsgPort(con_wr_port);
    if (con_rd_port)
        DeleteMsgPort(con_rd_port);
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
    con_puts(" 8N1, ANSI/Topaz\r\n"
             "Serial line ready - connect the bridge, then press Return.\r\n"
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
