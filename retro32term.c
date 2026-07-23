/*
 * Retro32 Terminal - a tiny serial ANSI terminal for the Retro32 BBS disk.
 *
 * Bridges the Amiga serial port to its own ANSI renderer on a 16-colour
 * hires screen, so whatever is on the other end of the serial line (the
 * Copperline browser bridge, --serial-connect, a null-modem cable) fills
 * the screen and the keyboard talks back. Earlier versions handed the
 * ANSI work to console.device; the console cannot express PC ANSI - SGR
 * 1 is a brightness bit on a PC but a font weight to the console, and
 * the console addresses only pens 0-7, leaving the bright palette half
 * (the dark grey of shadows and dot leaders above all) unreachable on
 * any Kickstart - so the terminal now parses and draws itself (see the
 * terminal engine section). Glyphs are Topaz 8, the font Amiga-oriented
 * BBSes such as Retro32 design for, pulled from the machine's own ROM
 * font at startup.
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
 * received byte, so every byte must be serviced within one character
 * time (~520 us at 19200) -- a tight budget on a stock 68000, because
 * the 4-bitplane hires display locks the CPU off the chip bus for most
 * of every scan line. Exec's interrupt dispatch plus a Signal() per
 * wakeup did not fit inside it (measured: scattered [LOST] markers on
 * Kickstart the moment the screen went 4 planes deep), so both
 * platforms point the level-5 autovector straight at the receive
 * handler and the main loops busy-poll instead of sleeping (see
 * lvl5_c). AROS needs the same vector for a second reason (its
 * dispatcher's INTREQ pre-ack race, see lvl5_c) and holds a permanent
 * Forbid() on top so its scheduler's ~1 ms interrupt-masked stretches
 * never run (see kiosk_loop). The defines stay separate as tuning
 * knobs. The Copperline bridge itself is not paced; this only sets how
 * fast the guest side reads and writes. */
#define BAUD_KICKSTART 19200UL
#define BAUD_AROS 19200UL

/* SERPER divisor from the PAL colour clock (an NTSC machine lands within
 * 1%, well inside the UART's tolerance at these rates). */
#define PAL_COLOR_CLOCK 3546895UL
#define SERPER_FOR(baud) ((UWORD)((PAL_COLOR_CLOCK + (baud) / 2) / (baud) - 1))

static ULONG baud;
static BOOL on_aros;

/* Opened in library mode (CONU_LIBRARY) purely for RawKeyConvert. */
struct Library *ConsoleDevice;

/* Received-byte ring between the RBF interrupt handler and the main loop.
 * Power of two, and it must divide 65536: the indices are UWORDs because
 * a 68000 reads/writes a word atomically but a longword as two accesses,
 * and each side reads the other side's index -- a torn 32-bit counter
 * silently drops or repeats bytes. The interrupt writes rx_head, the main
 * loop rx_tail; wrap-around arithmetic stays correct in 16 bits.
 *
 * Sized to absorb a whole BBS page, not just a burst: rendering runs
 * slower than the 19200 line rate on a stock 68000 (a glyph is cheap
 * but a scroll waits on a 4-plane blit), so the ring backlogs part of
 * a page while drawing catches up. 32 KiB is ~17 seconds of line-rate
 * data, comfortably above any screenful a BBS sends; the 8 KiB it
 * replaced overflowed mid-page on the larger Retro32 menus. */
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

/* The 16-colour CGA/VGA text palette in ANSI order (pen = SGR colour
 * for 30-37, plus 8 for the bright half): black, red, green, yellow,
 * blue, magenta, cyan, white, then their bright forms. Pen 3 is CGA
 * brown, pen 8 the dark grey this palette exists for. Values are
 * 12-bit 0x0RGB, one nibble per gun. */
static const UWORD ansi_rgb4[16] = {
    0x0000, 0x0A00, 0x00A0, 0x0A50, 0x000A, 0x0A0A, 0x00AA, 0x0AAA,
    0x0555, 0x0F55, 0x05F5, 0x0FF5, 0x055F, 0x0F5F, 0x05FF, 0x0FFF,
};

static struct Screen *screen;
static struct Window *window;

static ULONG old_lvl5;
static BOOL lvl5_taken;
static BOOL dsksyn_was_enabled;
static BOOL dtr_asserted;

/* Blank pointer sprite: control words + one bitplane row + terminator,
 * all zero. Must live in chip RAM and stay allocated while set. */
#define BLANK_POINTER_BYTES 12
static UWORD *blank_pointer;

/* --- Paula RBF interrupt ------------------------------------------------- */

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

/* The level-5 autovector (68000 vector 29) is pointed straight at this
 * handler on BOTH platforms; neither OS's dispatcher is compatible with
 * a one-byte hardware latch at sustained line rate:
 *
 * - AROS's exec acks INTREQ RBF BEFORE calling SetIntVector handlers,
 *   and that pre-ack is poison: with the request bit clear, Paula's
 *   receive latch is no longer protected, so a word completing between
 *   the dispatcher's ack and the handler's SERDATR read OVERWRITES the
 *   unread word -- silently, because overrun detection only triggers
 *   while the request bit is set.
 * - Kickstart's dispatch is correct but too slow once the 4-bitplane
 *   hires display owns the chip bus: the exec handler path plus a
 *   Signal() per interrupt runs a few hundred cycles on a CPU that is
 *   seeing mostly blanking slots, which overshoots the character time
 *   and overruns the same latch (seen as scattered [LOST] at 19200).
 *
 * With the direct vector the request bit stays set from word completion
 * until the ack below, the latch stays protected the whole way, and the
 * worst any race can do is an honest OVRUN that the [LOST] marker
 * reports. Banking the already-read value after the ack (rather than
 * re-reading) closes the overwrite race on the data itself. No OS calls
 * here: both main loops busy-poll, so not even Signal() is needed.
 * Level 5 also serves DSKSYN on Kickstart; setup parks that source (the
 * kiosk never touches the floppy again) and cleanup restores it. */
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

/* --- terminal engine ------------------------------------------------------
 * A PC-ANSI (ANSI.SYS / ANSI-BBS) terminal rendered by hand. Kickstart's
 * and AROS's console.device both fall short of what BBS art assumes:
 * SGR 1 selects a bold font instead of the bright palette half, only
 * pens 0-7 are addressable so bright black (grey) cannot exist, the
 * parameterized erase forms differ, and the AROS parser had to be
 * defended against outright (its command scanner runs past finals it
 * does not know, spilling the following text). So the console is not
 * used at all: incoming bytes run through a small ECMA-48 state machine
 * here and are drawn straight into the screen's 4 bitplanes.
 *
 * - A character cell is 8x8 pixels, and 8 hires pixels are exactly one
 *   byte per bitplane, so a glyph is 32 byte writes with foreground and
 *   background masks; no blitter, no read-modify-write.
 * - Glyphs come from the ROM Topaz 8 (see font_extract), so the art
 *   renders in the face it was drawn for on every Kickstart and AROS.
 * - Scrolls, erases and insert/delete go through BltBitMap on the
 *   screen bitmap: minterm 0xFF or 0x00 with a plane mask paints any
 *   pen, and BltBitMap picks the descending mode by itself when the
 *   rectangles overlap. The one rule is WaitBlit before the next CPU
 *   write into the planes (term_blit_sync).
 * - PC semantics throughout: the palette is the 16-colour CGA set in
 *   ANSI order, bold folds into bright foregrounds, blink into bright
 *   backgrounds (iCE colours), erases fill with the current background
 *   (BCE), ESC[2J homes the cursor like ANSI.SYS, autowrap is deferred
 *   DEC-style (a glyph in column 80 parks the cursor there and the
 *   next glyph wraps), and DSR 6 is answered on the serial line here,
 *   on Kickstart and AROS alike. */

#define COLS 80

static struct BitMap *term_bm; /* the screen's bitmap */
static UBYTE *term_plane[4];
static WORD term_bpr;  /* bytes per plane row (80 at 640 wide) */
static WORD term_rows; /* text rows: 32 PAL, 25 NTSC */

/* Topaz 8 glyphs, one byte per row, flat for fast cell addressing. */
static UBYTE font8[256 * 8];

static WORD cur_x, cur_y;
static WORD sav_x, sav_y; /* CSI s/u (and ESC 7/8) cursor save slot */
static WORD atr_fg = 7;   /* base colours 0-7; brightness lives in */
static WORD atr_bg;       /* atr_bold / atr_blink */
static WORD atr_bold, atr_blink, atr_inv, atr_under;
static WORD wrap_pending;
static WORD cursor_visible = 1; /* ESC[?25l/h */
static WORD cursor_drawn;       /* cursor cell is currently inverted */
static WORD blit_pending;       /* a BltBitMap has been started */

/* CSI parser state. */
#define P_MAX 16
static WORD p_state; /* 0 plain, 1 ESC, 2 ESC intermediates, 3 CSI body */
static WORD p_params[P_MAX];
static WORD p_np;     /* index of the parameter being collected */
static WORD p_have;   /* a digit or ';' has been seen */
static WORD p_priv;   /* leading private marker ('?'), or 0 */
static WORD p_ignore; /* unknown byte seen: parse fully, execute nothing */

/* Pull the glyph bitmaps out of an opened font. tf_CharData is one wide
 * bitmap, tf_Modulo bytes per scanline; tf_CharLoc gives each glyph's
 * bit offset into it. Topaz 8 is byte-aligned 8x8 so the shift path is
 * insurance, not the norm. Characters the font lacks stay blank (BSS). */
static void font_extract(struct TextFont *tf)
{
    const UBYTE *data = (const UBYTE *)tf->tf_CharData;
    const ULONG *loc = (const ULONG *)tf->tf_CharLoc;
    WORD ys = tf->tf_YSize > 8 ? 8 : tf->tf_YSize;
    WORD c, r;

    for (c = tf->tf_LoChar; c <= tf->tf_HiChar; c++) {
        ULONG bitoff = loc[c - tf->tf_LoChar] >> 16;
        const UBYTE *src = data + (bitoff >> 3);
        WORD sh = (WORD)(bitoff & 7);
        UBYTE *dst = &font8[c << 3];
        for (r = 0; r < ys; r++) {
            UBYTE g = (UBYTE)(src[0] << sh);
            if (sh)
                g |= src[1] >> (8 - sh);
            dst[r] = g;
            src += tf->tf_Modulo;
        }
    }
}

static void term_blit_sync(void)
{
    if (blit_pending) {
        WaitBlit();
        blit_pending = 0;
    }
}

/* Cell-aligned rectangle fill in an arbitrary pen: set the planes where
 * the pen has a 1 bit (minterm 0xFF ignores its inputs and writes 1s),
 * clear the rest. Coordinates and sizes are in character cells. */
static void term_rect_fill(WORD x, WORD y, WORD w, WORD h, WORD pen)
{
    ULONG setm = (ULONG)pen & 0xF;
    if (w <= 0 || h <= 0)
        return;
    if (setm)
        BltBitMap(term_bm, x << 3, y << 3, term_bm, x << 3, y << 3,
                  w << 3, h << 3, 0xFF, setm, NULL);
    if (setm != 0xF)
        BltBitMap(term_bm, x << 3, y << 3, term_bm, x << 3, y << 3,
                  w << 3, h << 3, 0x00, setm ^ 0xF, NULL);
    blit_pending = 1;
}

static void term_rect_copy(WORD sx, WORD sy, WORD dx, WORD dy, WORD w, WORD h)
{
    if (w <= 0 || h <= 0)
        return;
    BltBitMap(term_bm, sx << 3, sy << 3, term_bm, dx << 3, dy << 3,
              w << 3, h << 3, 0xC0, 0xF, NULL);
    blit_pending = 1;
}

static WORD term_fg_pen(void)
{
    WORD fg = atr_fg + (atr_bold ? 8 : 0);
    WORD bg = atr_bg + (atr_blink ? 8 : 0);
    return atr_inv ? bg : fg;
}

static WORD term_bg_pen(void)
{
    WORD fg = atr_fg + (atr_bold ? 8 : 0);
    WORD bg = atr_bg + (atr_blink ? 8 : 0);
    return atr_inv ? fg : bg;
}

static void term_glyph(WORD x, WORD y, UBYTE ch)
{
    const UBYTE *g = &font8[(UWORD)ch << 3];
    LONG off = (LONG)y * (term_bpr << 3) + x;
    WORD fg = term_fg_pen(), bg = term_bg_pen();
    WORD p, r;

    term_blit_sync();
    for (p = 0; p < 4; p++) {
        UBYTE fm = (UBYTE)((fg & (1 << p)) ? 0xFF : 0x00);
        UBYTE bm = (UBYTE)((bg & (1 << p)) ? 0xFF : 0x00);
        UBYTE *dst = term_plane[p] + off;
        for (r = 0; r < 8; r++) {
            UBYTE bits = g[r];
            if (r == 7 && atr_under)
                bits = 0xFF;
            *dst = (UBYTE)((bits & fm) | (~bits & bm));
            dst += term_bpr;
        }
    }
}

/* The cursor is the cell under it with every plane byte inverted (pen
 * XOR 15), which is self-restoring: flipping twice puts the cell back.
 * Drawing only ever happens with the cursor hidden (see drain_serial),
 * so glyphs never land on an inverted cell. */
static void term_cursor_flip(void)
{
    LONG off = (LONG)cur_y * (term_bpr << 3) + cur_x;
    WORD p, r;

    term_blit_sync();
    for (p = 0; p < 4; p++) {
        UBYTE *dst = term_plane[p] + off;
        for (r = 0; r < 8; r++) {
            *dst ^= 0xFF;
            dst += term_bpr;
        }
    }
}

static void cursor_hide(void)
{
    if (cursor_drawn) {
        term_cursor_flip();
        cursor_drawn = 0;
    }
}

static void cursor_show(void)
{
    if (cursor_visible && !cursor_drawn) {
        term_cursor_flip();
        cursor_drawn = 1;
    }
}

static void term_scroll_up(WORD n)
{
    WORD bg = term_bg_pen();
    if (n >= term_rows) {
        term_rect_fill(0, 0, COLS, term_rows, bg);
        return;
    }
    term_rect_copy(0, n, 0, 0, COLS, term_rows - n);
    term_rect_fill(0, term_rows - n, COLS, n, bg);
}

static void term_scroll_down(WORD n)
{
    WORD bg = term_bg_pen();
    if (n >= term_rows) {
        term_rect_fill(0, 0, COLS, term_rows, bg);
        return;
    }
    term_rect_copy(0, 0, 0, n, COLS, term_rows - n);
    term_rect_fill(0, 0, COLS, n, bg);
}

static void term_erase_display(WORD mode)
{
    WORD bg = term_bg_pen();
    if (mode >= 2) {
        /* ANSI.SYS semantics: 2J (and xterm's 3J) clears and homes. */
        term_rect_fill(0, 0, COLS, term_rows, bg);
        cur_x = cur_y = 0;
        wrap_pending = 0;
    } else if (mode == 1) {
        term_rect_fill(0, 0, COLS, cur_y, bg);
        term_rect_fill(0, cur_y, cur_x + 1, 1, bg);
    } else {
        term_rect_fill(cur_x, cur_y, COLS - cur_x, 1, bg);
        term_rect_fill(0, cur_y + 1, COLS, term_rows - 1 - cur_y, bg);
    }
}

static void term_erase_line(WORD mode)
{
    WORD bg = term_bg_pen();
    if (mode >= 2)
        term_rect_fill(0, cur_y, COLS, 1, bg);
    else if (mode == 1)
        term_rect_fill(0, cur_y, cur_x + 1, 1, bg);
    else
        term_rect_fill(cur_x, cur_y, COLS - cur_x, 1, bg);
}

static void term_insert_lines(WORD n)
{
    WORD below = term_rows - cur_y;
    if (n > below)
        n = below;
    term_rect_copy(0, cur_y, 0, cur_y + n, COLS, below - n);
    term_rect_fill(0, cur_y, COLS, n, term_bg_pen());
}

static void term_delete_lines(WORD n)
{
    WORD below = term_rows - cur_y;
    if (n > below)
        n = below;
    term_rect_copy(0, cur_y + n, 0, cur_y, COLS, below - n);
    term_rect_fill(0, term_rows - n, COLS, n, term_bg_pen());
}

static void term_insert_chars(WORD n)
{
    WORD rest = COLS - cur_x;
    if (n > rest)
        n = rest;
    term_rect_copy(cur_x, cur_y, cur_x + n, cur_y, rest - n, 1);
    term_rect_fill(cur_x, cur_y, n, 1, term_bg_pen());
}

static void term_delete_chars(WORD n)
{
    WORD rest = COLS - cur_x;
    if (n > rest)
        n = rest;
    term_rect_copy(cur_x + n, cur_y, cur_x, cur_y, rest - n, 1);
    term_rect_fill(COLS - n, cur_y, n, 1, term_bg_pen());
}

static void term_sgr(WORD v)
{
    /* aixterm bright forms: brightness without the attribute dance. */
    if (v >= 90 && v <= 97) {
        atr_fg = v - 90;
        atr_bold = 1;
        return;
    }
    if (v >= 100 && v <= 107) {
        atr_bg = v - 100;
        atr_blink = 1;
        return;
    }
    switch (v) {
    case 0:
        atr_fg = 7;
        atr_bg = 0;
        atr_bold = atr_blink = atr_inv = atr_under = 0;
        break;
    case 1:
        atr_bold = 1;
        break;
    case 2: /* faint: no separate rendition, treat as intensity off */
    case 21:
    case 22:
        atr_bold = 0;
        break;
    case 4:
        atr_under = 1;
        break;
    case 24:
        atr_under = 0;
        break;
    case 5: /* blink selects bright backgrounds, the iCE colour rule */
    case 6:
        atr_blink = 1;
        break;
    case 25:
        atr_blink = 0;
        break;
    case 7:
        atr_inv = 1;
        break;
    case 27:
        atr_inv = 0;
        break;
    case 39:
        atr_fg = 7;
        break;
    case 49:
        atr_bg = 0;
        break;
    default:
        if (v >= 30 && v <= 37)
            atr_fg = v - 30;
        else if (v >= 40 && v <= 47)
            atr_bg = v - 40;
        break;
    }
}

static void term_linefeed(void)
{
    if (cur_y >= term_rows - 1) {
        cur_y = term_rows - 1;
        term_scroll_up(1);
    } else {
        cur_y++;
    }
}

static void term_ctl(UBYTE b)
{
    switch (b) {
    case 0x08: /* BS: move, do not erase */
        if (cur_x > 0)
            cur_x--;
        wrap_pending = 0;
        break;
    case 0x09: /* TAB: 8-column stops, clamped to the last column */
        cur_x = (cur_x & ~7) + 8;
        if (cur_x > COLS - 1)
            cur_x = COLS - 1;
        wrap_pending = 0;
        break;
    case 0x0A:
        wrap_pending = 0;
        term_linefeed();
        break;
    case 0x0C: /* FF: clear and home, as on the console it replaces */
        term_rect_fill(0, 0, COLS, term_rows, term_bg_pen());
        cur_x = cur_y = 0;
        wrap_pending = 0;
        break;
    case 0x0D:
        cur_x = 0;
        wrap_pending = 0;
        break;
    default: /* BEL and the rest: ignored */
        break;
    }
}

static void term_printable(UBYTE ch)
{
    if (wrap_pending) {
        wrap_pending = 0;
        cur_x = 0;
        term_linefeed();
    }
    term_glyph(cur_x, cur_y, ch);
    if (cur_x >= COLS - 1)
        wrap_pending = 1;
    else
        cur_x++;
}

static WORD fmt_num(UBYTE *buf, WORD n, WORD v)
{
    UBYTE d[5];
    WORD k = 0;
    if (!v)
        d[k++] = '0';
    while (v) {
        d[k++] = (UBYTE)('0' + v % 10);
        v /= 10;
    }
    while (k)
        buf[n++] = d[--k];
    return n;
}

/* DSR: answered here on the serial line, so it works identically on
 * Kickstart and AROS (the console used to answer it on Kickstart only). */
static void term_dsr(WORD which)
{
    UBYTE buf[12];
    WORD n = 0;
    if (which == 6) { /* CPR: cursor position, 1-based */
        buf[n++] = 0x1B;
        buf[n++] = '[';
        n = fmt_num(buf, n, cur_y + 1);
        buf[n++] = ';';
        n = fmt_num(buf, n, cur_x + 1);
        buf[n++] = 'R';
        ser_write(buf, n);
    } else if (which == 5) { /* device status: ready */
        ser_write((const UBYTE *)"\x1B[0n", 4);
    }
}

static WORD pn1(WORD i)
{
    return p_params[i] ? p_params[i] : 1;
}

/* Shared tail for every cursor motion: absolute and relative moves both
 * clamp to the screen and cancel a pending wrap. */
static void term_moved(void)
{
    if (cur_x < 0)
        cur_x = 0;
    if (cur_x > COLS - 1)
        cur_x = COLS - 1;
    if (cur_y < 0)
        cur_y = 0;
    if (cur_y > term_rows - 1)
        cur_y = term_rows - 1;
    wrap_pending = 0;
}

static void term_reset(void);

static void term_csi(UBYTE final)
{
    WORD i, np;

    if (p_priv) {
        /* DEC private modes: only cursor visibility (ESC[?25l/h)
         * matters to BBS output; the rest parse and drop. */
        if (p_params[0] == 25 && (final == 'h' || final == 'l'))
            cursor_visible = (final == 'h');
        return;
    }
    if (p_ignore)
        return;

    switch (final) {
    case 'A':
        cur_y -= pn1(0);
        term_moved();
        break;
    case 'B':
        cur_y += pn1(0);
        term_moved();
        break;
    case 'C':
        cur_x += pn1(0);
        term_moved();
        break;
    case 'D':
        cur_x -= pn1(0);
        term_moved();
        break;
    case 'E':
        cur_x = 0;
        cur_y += pn1(0);
        term_moved();
        break;
    case 'F':
        cur_x = 0;
        cur_y -= pn1(0);
        term_moved();
        break;
    case 'G':
        cur_x = pn1(0) - 1;
        term_moved();
        break;
    case 'f': /* HVP: same motion as CUP */
    case 'H':
        cur_y = pn1(0) - 1;
        cur_x = pn1(1) - 1;
        term_moved();
        break;
    case 'J':
        term_erase_display(p_params[0]);
        break;
    case 'K':
        term_erase_line(p_params[0]);
        break;
    case 'L':
        term_insert_lines(pn1(0));
        break;
    case 'M':
        term_delete_lines(pn1(0));
        break;
    case '@':
        term_insert_chars(pn1(0));
        break;
    case 'P':
        term_delete_chars(pn1(0));
        break;
    case 'S':
        term_scroll_up(pn1(0));
        break;
    case 'T':
        term_scroll_down(pn1(0));
        break;
    case 'm':
        np = p_np + 1;
        for (i = 0; i < np; i++) {
            WORD v = p_params[i];
            if (v == 38 || v == 48) {
                /* Extended colour introducers carry sub-arguments
                 * (38;5;N, 38;2;R;G;B) that must be consumed, not
                 * executed - N would otherwise read as blink or the
                 * like. 16 pens have nothing to map them onto, so
                 * they are skipped whole. */
                if (i + 1 < np && p_params[i + 1] == 5)
                    i += 2;
                else if (i + 1 < np && p_params[i + 1] == 2)
                    i += 4;
                continue;
            }
            term_sgr(v);
        }
        break;
    case 's':
        sav_x = cur_x;
        sav_y = cur_y;
        break;
    case 'u':
        cur_x = sav_x;
        cur_y = sav_y;
        term_moved();
        break;
    case 'n':
        term_dsr(p_params[0]);
        break;
    default:
        /* Anything else: a real terminal parses and ignores. */
        break;
    }
}

static void term_csi_begin(void)
{
    WORD i;
    for (i = 0; i < P_MAX; i++)
        p_params[i] = 0;
    p_np = 0;
    p_have = 0;
    p_priv = 0;
    p_ignore = 0;
    p_state = 3;
}

/* One byte of BBS output. */
static void term_feed(UBYTE b)
{
    switch (p_state) {
    case 1: /* ESC seen */
        if (b == '[') {
            term_csi_begin();
            return;
        }
        if (b == 0x1B)
            return; /* ESC restarts ESC */
        if (b >= 0x20 && b <= 0x2F) {
            p_state = 2;
            return;
        }
        p_state = 0;
        if (b == 'c')
            term_reset(); /* RIS */
        else if (b == '7') {
            sav_x = cur_x;
            sav_y = cur_y;
        } else if (b == '8') {
            cur_x = sav_x;
            cur_y = sav_y;
            term_moved();
        }
        /* every other ESC sequence: parsed and dropped */
        return;
    case 2: /* ESC intermediates: swallow through the final byte */
        if (b >= 0x30 && b <= 0x7E)
            p_state = 0;
        return;
    case 3: /* CSI body */
        if (b >= '0' && b <= '9') {
            WORD v = p_params[p_np];
            if (v < 1000)
                p_params[p_np] = v * 10 + (b - '0');
            p_have = 1;
        } else if (b == ';') {
            if (p_np < P_MAX - 1)
                p_np++;
            p_have = 1;
        } else if (b == '?' && !p_have && !p_priv) {
            p_priv = b;
        } else if (b == 0x1B) {
            p_state = 1; /* aborted sequence, ESC restarts */
        } else if (b == 0x9B) {
            term_csi_begin(); /* ditto for the 8-bit introducer */
        } else if (b < 0x20) {
            term_ctl(b); /* ECMA-48: controls execute mid-sequence */
        } else if (b >= 0x40 && b <= 0x7E) {
            p_state = 0;
            term_csi(b);
        } else {
            /* other private markers, intermediates, ':' subparameters
             * (ESC[<...m, ESC[0;40 D, ESC[38:5:201m): parse to the
             * final byte, execute nothing */
            p_ignore = 1;
        }
        return;
    default:
        break;
    }

    /* plain data */
    if (b == 0x1B) {
        p_state = 1;
        return;
    }
    if (b == 0x9B) {
        term_csi_begin();
        return;
    }
    if (b < 0x20) {
        term_ctl(b);
        return;
    }
    term_printable(b);
}

static void term_puts(const char *s)
{
    while (*s)
        term_feed((UBYTE)*s++);
}

static void term_num(ULONG v)
{
    UBYTE d[10];
    WORD k = 0;
    if (!v)
        d[k++] = '0';
    while (v) {
        d[k++] = (UBYTE)('0' + v % 10);
        v /= 10;
    }
    while (k)
        term_feed(d[--k]);
}

/* RIS (ESC c), and the power-on state. */
static void term_reset(void)
{
    term_sgr(0);
    cur_x = cur_y = sav_x = sav_y = 0;
    wrap_pending = 0;
    cursor_visible = 1;
    cursor_drawn = 0;
    term_rect_fill(0, 0, COLS, term_rows, 0);
}

static int term_init(void)
{
    WORD i;

    /* RastPort.BitMap is the authoritative bitmap on every OS (the
     * embedded Screen.BitMap is a compatibility copy on V39+/AROS). */
    term_bm = screen->RastPort.BitMap;
    if (term_bm->Depth < 4)
        return 1;
    for (i = 0; i < 4; i++) {
        term_plane[i] = term_bm->Planes[i];
        if (!term_plane[i])
            return 1;
    }
    term_bpr = (WORD)term_bm->BytesPerRow;
    term_rows = screen->Height >> 3;
    term_reset();
    return 0;
}

/* Debug aid: dump incoming bytes as hex instead of interpreting them. */
#define DEBUG_HEX 0

/* Interpret everything the interrupt handler has banked, bounded so a
 * long backlog cannot starve the keyboard poll (the kiosk loop calls
 * straight back in; the Kickstart loop re-drains before Wait). */
#define DRAIN_BUDGET 4096

static void drain_serial(void)
{
    LONG budget = DRAIN_BUDGET;

    if (rx_tail == rx_head && !rx_lost)
        return;
    cursor_hide();
    if (rx_lost) {
        /* Show the loss, but do not disturb the BBS's rendition: the
         * attribute state is plain variables now, so save and restore
         * them around the marker instead of splicing escapes into the
         * stream. */
        WORD fg = atr_fg, bg = atr_bg, bo = atr_bold, bl = atr_blink;
        WORD iv = atr_inv, un = atr_under;
        rx_lost = 0;
        term_puts("\x1B[0;41;37m[LOST]");
        atr_fg = fg;
        atr_bg = bg;
        atr_bold = bo;
        atr_blink = bl;
        atr_inv = iv;
        atr_under = un;
    }
    while (rx_tail != rx_head && budget--) {
#if DEBUG_HEX
        static const char hexd[] = "0123456789abcdef";
        UBYTE b = ring[rx_tail % RING];
        term_printable(hexd[b >> 4]);
        term_printable(hexd[b & 15]);
        term_printable(' ');
#else
        term_feed(ring[rx_tail % RING]);
#endif
        rx_tail++;
    }
    cursor_show();
}

/* Keystroke, translated: forward it down the line. RawKeyConvert emits
 * 8-bit CSI (0x9B) for special keys; BBSes expect the 7-bit ESC [ form,
 * so rewrite that one byte and pass everything else verbatim. */
static void forward_key(UBYTE c)
{
    static const UBYTE esc_bracket[2] = { 0x1B, '[' };
    if (c == 0x9B)
        ser_write(esc_bracket, 2);
    else
        ser_write(&c, 1);
}

/* Rawkey down-event to bytes on the wire, shared by both keyboard
 * paths (IDCMP on Kickstart, the CIA poll in the AROS kiosk). */
static void key_convert(UWORD code, UWORD qual)
{
    struct InputEvent ie;
    UBYTE buf[16];
    LONG n, i;

    ie.ie_NextEvent = NULL;
    ie.ie_Class = IECLASS_RAWKEY;
    ie.ie_SubClass = 0;
    ie.ie_Code = code;
    ie.ie_Qualifier = qual;
    ie.ie_EventAddress = 0;
    ie.ie_Seconds = 0;
    ie.ie_Micros = 0;
    n = RawKeyConvert(&ie, buf, (LONG)sizeof(buf), NULL);
    for (i = 0; i < n; i++)
        forward_key(buf[i]);
}

/* Kickstart keyboard input: RAWKEY IDCMP messages from our window,
 * translated through the same keymap path as the kiosk. input.device
 * stays alive on Kickstart, so key repeat works here. */
static void handle_idcmp(void)
{
    struct IntuiMessage *im;
    while ((im = (struct IntuiMessage *)GetMsg(WINDOW_USERPORT(window)))) {
        ULONG cls = im->Class;
        UWORD code = im->Code, qual = im->Qualifier;
        ReplyMsg(&im->ExecMessage);
        if (cls == IDCMP_RAWKEY && !(code & IECODE_UP_PREFIX))
            key_convert(code, qual);
    }
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
 * - Rendering does not care: the terminal engine writes the bitplanes
 *   with the CPU and calls BltBitMap, which runs the blit in the
 *   caller's context; no other task is involved.
 * - Console CMD_READ would never complete, and IDCMP messages need
 *   input.device's task, so the keyboard is read from the CIA-A
 *   hardware here instead: INTENA's PORTS bit is cleared so the OS
 *   keyboard handler no longer swallows the scancodes, then the loop
 *   polls the SP flag, takes the scancode from SDR, performs the KDAT
 *   handshake, and feeds the code through console.device's
 *   RawKeyConvert() -- on AROS a synchronous keymap.library call, safe
 *   under Forbid. Key repeat was input.device's job, so kiosk mode has
 *   none. */

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
    key_convert(code, kbd_qual);
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

/* AROS's m68k WaitBlit (arch/m68k-amiga/graphics/waitblit.S) sets
 * blitter-nasty (DMACON BLTPRI) for the duration of every wait and
 * parks the CPU off the chip bus until the blit completes. With no
 * fast RAM the RBF handler's instruction fetches stall with it, so a
 * blit longer than a character time (~520 us at 19200; the engine's
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

    for (;;) {
        drain_serial();
        kbd_poll();
    }
}

/* Pins the 8x8 Topaz everywhere it matters: the screen's own font (a
 * bare-floppy Kickstart boot can default to a wider Topaz; observed
 * 10x9 on 3.1) and the OpenFont in setup that the glyph extraction
 * reads. BBS pages are drawn for 80 columns of this face. */
static struct TextAttr topaz8 = { "topaz.font", 8, 0, 0 };

/* Exec's CreateMsgPort/CreateIORequest are V36+ and this program
 * otherwise runs on Kickstart 1.3, so the port and request are static
 * structures initialized by hand. The port is private (never
 * AddPort'ed), which is all CreateMsgPort produced here anyway; the
 * signal bit is the only real allocation. */
static struct MsgPort port_con;
static struct IOStdReq ioreq_con;
static struct MsgPort *con_port;
static struct IOStdReq *con_req;
static BOOL con_open;

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
    struct ColorSpec colors[17];
    struct TagItem screen_tags[3];
    struct NewScreen ns;
    struct NewWindow nw;
    struct TextFont *tf;
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
    for (i = 0; i < 16; i++) {
        colors[i].ColorIndex = i;
        colors[i].Red = (ansi_rgb4[i] >> 8) & 0xF;
        colors[i].Green = (ansi_rgb4[i] >> 4) & 0xF;
        colors[i].Blue = ansi_rgb4[i] & 0xF;
    }
    colors[16].ColorIndex = -1;
    colors[16].Red = colors[16].Green = colors[16].Blue = 0;

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
     * pattern -- when intuition is new enough to take tags at all. Four
     * bitplanes for the 16 ANSI colours: hires allows exactly 4 planes
     * on OCS/ECS, so this needs no AGA anywhere -- but the 80 KiB
     * bitmap must be chip RAM (bitplane DMA cannot reach the $C00000
     * trapdoor slow RAM), and an AROS boot in tight configs does not
     * leave that much chip free; there is deliberately no shallower
     * fallback, the terminal fails with a message instead of degrading
     * the palette. PAL rows first, NTSC rows if the machine cannot do
     * 256. */
    for (i = 0; i < (WORD)sizeof(ns); i++)
        ((UBYTE *)&ns)[i] = 0;
    ns.Width = 640;
    ns.Depth = 4;
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
    nw.IDCMPFlags = IDCMP_RAWKEY; /* the Kickstart keyboard path */
    nw.Flags = WFLG_SIMPLE_REFRESH | WFLG_BACKDROP | WFLG_BORDERLESS
        | WFLG_ACTIVATE | WFLG_RMBTRAP;
    nw.Screen = screen;
    nw.Type = CUSTOMSCREEN;
    window = OpenWindow(&nw);
    if (!window)
        return 4;

    /* A V34 intuition ignored the tags, so do their work by hand: drop
     * the title bar behind the backdrop window and load the palette.
     * The OpenScreen-applies-it ordering that SA_Colors exists for
     * only matters on AROS, which is always V36+; on Kickstart a
     * LoadRGB4 after the fact sticks. */
    if (!v36) {
        ShowTitle(screen, FALSE);
        LoadRGB4(ViewPortAddress(window), ansi_rgb4, 16);
    }

    /* The kiosk takes no pointer input, so hide the Intuition pointer:
     * a blank 1-row sprite (chip RAM, zeroed) instead of the busy/arrow
     * imagery darting over the text whenever the host mouse moves.
     * Failing to allocate just leaves the normal pointer - cosmetic. */
    blank_pointer = AllocMem(BLANK_POINTER_BYTES, MEMF_CHIP | MEMF_CLEAR);
    if (blank_pointer)
        SetPointer(window, blank_pointer, 1, 16, 0, 0);

    /* The engine's glyphs: the machine's own ROM Topaz 8. */
    tf = OpenFont(&topaz8);
    if (!tf)
        return 5;
    font_extract(tf);
    CloseFont(tf);

    if (term_init())
        return 6;

    /* console.device in library mode: no window, no unit task, just
     * the RawKeyConvert vector both keyboard paths translate with. */
    con_port = port_init(&port_con);
    if (!con_port)
        return 7;
    con_req = ioreq_init(&ioreq_con, con_port);
    if (OpenDevice("console.device", CONU_LIBRARY,
                   (struct IORequest *)con_req, 0))
        return 8;
    con_open = TRUE;
    ConsoleDevice = (struct Library *)con_req->io_Device;

    /* Scrolls and fills are big blits; with BLTPRI (blitter-hog) set
     * they lock the CPU off the chip bus for milliseconds at a time,
     * which is longer than a character time and overruns Paula's
     * one-byte receive buffer no matter how fast the RBF handler is.
     * This disk is a dedicated terminal, so trade a slower blit for a
     * live UART. */
    CUSTOM_DMACON = DMAF_BLITHOG;

    /* AROS exposes a resident no Kickstart has; see the baud note at the
     * top of the file and kiosk_loop. */
    on_aros = FindResident("aros.library") != NULL;
    baud = on_aros ? BAUD_AROS : BAUD_KICKSTART;
    CUSTOM_SERPER = SERPER_FOR(baud);

    /* Take over the serial hardware: point the level-5 autovector at
     * the receive handler (see lvl5_c for why neither OS's dispatcher
     * can be used). Level 5 also serves DSKSYN on Kickstart, so that
     * source is parked for the duration -- the terminal never touches
     * the floppy again -- and restored on exit. RBF is masked around
     * the swap so the interrupt cannot fire between the two word
     * writes of the 32-bit vector. */
    dsksyn_was_enabled = (CUSTOM_INTENAR & INTF_DSKSYN) != 0;
    CUSTOM_INTENA = INTF_RBF | INTF_DSKSYN;
    old_lvl5 = VEC_LEVEL5;
    VEC_LEVEL5 = (ULONG)lvl5_stub;
    lvl5_taken = TRUE;
    CUSTOM_INTREQ = INTF_RBF | INTF_DSKSYN;         /* clear stale */
    CUSTOM_INTENA = INTF_SETCLR | INTF_INTEN | INTF_RBF;

    /* Raise DTR and RTS the way serial.device's Open does: whatever is on
     * the far end of the wire keys off DTR to know a terminal is ready.
     * The Copperline browser bridge defers its BBS dial until the guest
     * asserts DTR, so raising it only here - after the renderer and the RBF
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
    if (lvl5_taken) {
        CUSTOM_INTENA = INTF_RBF; /* disable RBF, leave the master alone */
        CUSTOM_INTREQ = INTF_RBF;
        VEC_LEVEL5 = old_lvl5;
        if (dsksyn_was_enabled)
            CUSTOM_INTENA = INTF_SETCLR | INTF_DSKSYN;
    }
    if (con_open)
        CloseDevice((struct IORequest *)con_req);
    /* The port and request are static; only the port's signal bit
     * was allocated. */
    if (con_port)
        FreeSignal(con_port->mp_SigBit);
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
    {
        int rc = setup();
        if (rc) {
            if (rc == 3)
                printf("term: cannot open the 16-colour screen - the 80K\n"
                       "4-bitplane bitmap needs free CHIP RAM (slow/fast\n"
                       "RAM cannot hold bitplanes). Give the machine more\n"
                       "chip RAM (1M chip on an A500-class config).\n");
            else
                printf("term: setup failed (step %d)\n", rc);
            cleanup();
            return 20;
        }
    }

    term_puts("\x1B[1;37mRetro32 Terminal\x1B[0m  ");
    term_num(baud);
    /* DTR is up (setup raised it), so a bridge armed with a deferred
     * dial connects right about now and the far end's greeting simply
     * appears. Do not invite a blind Return: at a BBS login prompt an
     * unsolicited Return reads as an empty name and starts the new-user
     * flow. */
    term_puts(" 8N1, ANSI 16-colour, Topaz\r\n"
              "Serial line ready - click Connect on the page if you have not already.\r\n"
              "\r\n");
    cursor_show();

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

    /* Busy-poll like the kiosk does: the receive handler no longer
     * signals anybody (see lvl5_c), and a dedicated terminal has
     * nothing better to spend the CPU on. Unlike the kiosk there is
     * no Forbid here, so input.device stays live at higher priority
     * and IDCMP keys (with repeat) keep arriving. */
    for (;;) {
        drain_serial();
        handle_idcmp();
        if (SetSignal(0, 0) & SIGBREAKF_CTRL_C)
            break;
    }

    cleanup();
    return 0;
}
