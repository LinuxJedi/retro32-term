# retro32-term

An Amiga ANSI terminal emulator for serial connections: a bootable
floppy that goes straight into a serial terminal, built for dialling
the [Retro32 BBS](https://www.retro32.com/retro32-bbs) from the
[Copperline](https://github.com/LinuxJedi/Copperline) Amiga emulator's
browser build, and equally usable on any Amiga (or emulator) whose
serial port is wired to a telnet bridge, null modem, or real modem.

The visitor flow it was designed for: a web page boots this disk in
the in-browser emulator, the visitor clicks Connect, presses Return,
and the BBS banner appears.

## What is on the disk

- `term` -- a small serial terminal (`retro32term.c`): drives Paula's
  UART directly (SERPER/SERDAT/SERDATR plus an RBF interrupt handler)
  and renders PC ANSI itself on its own 16-colour hires screen.
  Four bitplanes at 640 wide is legal on plain OCS, so the full CGA
  palette needs no AGA anywhere. The palette is handed to intuition
  with SA_Colors -- the only ordering that reliably beats the OS
  default palette everywhere (AROS loads its preference colours onto
  every new screen) -- with SA_ShowTitle off so the screen bar stays
  behind the backdrop window. On a V34 intuition (Kickstart 1.3),
  which takes no tags, the same is done by hand with LoadRGB4 and
  ShowTitle after the screen opens. Glyphs are the machine's own ROM
  Topaz 8 -- the font Amiga-oriented BBSes design for -- extracted at
  startup. Special keys (arrows) go out as ESC [ sequences.
- `s/startup-sequence` -- just runs `term`.

Earlier versions rendered through console.device with a translation
shim in front of it. That was abandoned: the console cannot express PC
ANSI. SGR 1 is a brightness bit on a PC (`1;30` is the dark grey BBS
art uses for shadows and dot leaders) but a font weight to the console,
which smears the glyphs and leaves bright black as black-on-black; and
the console addresses only pens 0-7 on every Kickstart, so the bright
palette half is structurally unreachable no matter how deep the screen
is. The AROS console additionally had to be defended against outright
(its command scanner runs past finals it does not know and prints the
following output as literal text). The terminal now parses ECMA-48
itself and draws straight into the bitplanes:

- A character cell is 8x8 pixels, and 8 hires pixels are one byte per
  bitplane, so a glyph is 32 byte-aligned CPU writes with foreground
  and background masks. Scrolls, erases and insert/delete run through
  `BltBitMap` (minterm 0xFF/0x00 with a plane mask paints any pen);
  the one rule is `WaitBlit` before the next CPU write to the planes.
- PC semantics throughout: the 16-colour CGA palette in ANSI order
  (pen 8 is the dark grey the rewrite exists for), bold folds into
  bright foregrounds and blink into bright backgrounds (iCE colours),
  erases fill with the current background (BCE), `ESC[2J` homes the
  cursor like ANSI.SYS, and autowrap is deferred DEC-style (a glyph
  in column 80 parks the cursor; the next glyph wraps), so both
  80-column art styles render correctly.
- Cursor motion, erase (including the backwards forms), CHA, HVP,
  insert/delete character and line, scroll region-less SU/SD,
  save/restore cursor, and `ESC[?25l/h` cursor visibility are all
  implemented; DSR 6 (cursor position report) is answered on the
  serial line directly, on Kickstart and AROS alike.
- Extended-colour introducers (`38;5;N`, `38;2;R;G;B`) consume their
  sub-arguments and drop them; everything else unknown -- private
  modes, intermediate bytes, colon subparameters, window ops -- is
  parsed to its final byte and ignored, exactly as a real terminal
  does.

The receive path takes the 68000 level-5 autovector directly on BOTH
platforms -- neither OS's interrupt dispatch is compatible with a
one-byte hardware latch at sustained line rate. AROS acks INTREQ
before dispatching `SetIntVector` handlers, which unprotects Paula's
receive latch and lets a completing word silently overwrite the unread
one. Kickstart dispatches correctly but too slowly once the
4-bitplane hires display owns the chip bus: exec's handler path plus a
`Signal()` per interrupt runs a few hundred cycles on a CPU that only
sees blanking slots, which overshoots the ~520 us character time
(measured: scattered `[LOST]` at 19200 the moment the screen went 4
planes deep). With the direct vector the handler banks the latched
word with the request bit still protecting it, and both main loops
busy-poll instead of sleeping, so no OS call ever sits between a byte
arriving and being banked. Level 5 also serves DSKSYN on Kickstart;
the terminal parks that source (it never touches the floppy again)
and restores it on exit. The ring indices between the handler and the
main loop are UWORDs: a 68000 writes a word atomically but a longword
as two accesses, and a torn 32-bit counter silently drops bytes.

Line settings: 8N1, 19200 on both Kickstart and AROS. AROS needs one
more measure: its scheduler masks all interrupts in roughly
millisecond stretches -- longer than a 19200 character time. Those
masked stretches only happen when the interrupt-exit path invokes the
scheduler (`core_ExitIntr` in AROS's
`arch/m68k-all/kernel/kernel_intr.c`), and that is skipped whenever
task switching is disabled, so on AROS (detected via
`FindResident("aros.library")`) the loop holds `Forbid()`
permanently. Rendering does not mind -- the engine writes the
bitplanes with the CPU and `BltBitMap` runs in the caller's context --
but keyboard input needs a live input.device, so under Forbid the
keyboard is polled straight from CIA-A (scancode from SDR, KDAT
handshake by beam-counter timing) and translated through
console.device's synchronous `RawKeyConvert()` (the device is opened
in `CONU_LIBRARY` mode purely for that vector). On Kickstart the same
translation is fed from RAWKEY IDCMP messages, which also keeps key
repeat. The trade-offs: on AROS no other task ever runs again (fine,
it is a kiosk; Ctrl-Amiga-Amiga still reboots) and held keys do not
repeat there (that was input.device's job). Change the `BAUD_*`
defines and rebuild to tune. The program also clears the DMACON
blitter-hog bit at startup, because scrolls and fills are big blits
that would otherwise lock the CPU off the chip bus for longer than a
character time.

Clearing blitter-hog is not enough on AROS, though: its m68k
`WaitBlit` (`arch/m68k-amiga/graphics/waitblit.S`) re-asserts
blitter-nasty for the duration of every wait and deliberately parks
the CPU off the chip bus until the blit completes. With no fast RAM,
the RBF handler's instruction fetches stall with it, so any screen
blit longer than a character time silently overran Paula's one-byte
receive latch (measured: ~12 lost bytes per 10 KiB page, surfacing as
scattered mangled characters). The kiosk therefore `SetFunction()`s
graphics.library `WaitBlit` with a plain busy-spin that leaves
blitter-nasty alone: blits finish a little slower with the CPU
sharing the bus, and reception stays byte-perfect (verified with
emulator-side overrun instrumentation: zero drops across the same
replayed page).

The receive ring between the interrupt handler and the renderer is
32 KiB -- about 17 seconds of line-rate data. That is deliberate:
rendering runs slower than the 19200 line rate on a stock 68000
(glyphs are cheap, but scrolls wait on 4-plane blits), so part of a
large BBS page sits in the ring while drawing catches up, and the
ring must hold a whole page (the 8 KiB it replaced
overflowed mid-page on the larger Retro32 menus, which surfaced as
mangled text in the second half of the page). On overflow the handler
now drops the new byte cleanly -- the write pointer must not step past
an unwritten slot, or the drain loop replays whatever that slot held a
lap earlier, silently substituting stale bytes into the stream. If
receive data is ever provably lost (a Paula overrun caught by the
handler, or the ring overflowing), a red `[LOST]` marker is printed
instead of failing silently.

Why not serial.device? Kickstart 2.0+ keeps it on the Workbench disk
(`DEVS:`), and the AROS ROM has no openable serial.device from a bare
boot floppy either -- a self-contained boot disk cannot rely on it.
Driving the hardware works identically everywhere and is what a
dedicated kiosk disk should do anyway.

## ROM compatibility

The 16-colour renderer build is verified byte-perfect at 19200 under
Copperline on Kickstart 3.1 and the AROS ROM (colour test page plus an
80-line coloured-ruler blast that scrolls the whole screen at
sustained line rate). Sizing note for AROS: the 80 KiB 4-plane bitmap
must live in chip RAM -- bitplane DMA cannot reach the $C00000
trapdoor slow RAM, and the terminal's other allocations (binary, ring,
font) already land in slow RAM by exec's own preference -- and an AROS
boot does not leave 80 KiB of chip free in tight configs. Verified
working for 16 colours: the default A500 machine with `--chip 1M`
(which keeps the 512 KiB trapdoor slow RAM, so AROS's own allocations
stay out of chip). NOT sufficient: 512 KiB chip, and also the A500+
profile (1 MB chip but no slow RAM -- AROS then lives entirely in chip
and the screen open still fails). There is deliberately no shallower
screen fallback: when the open fails the terminal exits with a message
saying chip RAM is what it needs. Kickstart boots run 16-colour in
stock 512K chip + 512K slow.

- **Kickstart 2.0+ (2.05 and 3.1 verified): byte-perfect, full ANSI
  colour on black, 19200 baud.**
- **The AROS m68k ROM Copperline bundles: byte-perfect, full ANSI
  colour on black, 19200 baud** via the permanent-Forbid kiosk mode
  described above (verified against a 6.6 KiB continuous back-to-back
  blast: zero dropped bytes, where the pre-kiosk `Wait()` loop at
  19200 visibly shreds the same stream). Needs Copperline v0.13+ (the
  Paula receive fixes in [Copperline PR
  #218](https://github.com/LinuxJedi/Copperline/pull/218)). AROS-only
  footnotes: the kernel writes its boot log to the serial line before
  the terminal starts -- a real BBS dialled after boot never sees it,
  and the browser bridge connects after boot anyway -- and held keys
  do not repeat.
- **Kickstart 1.3: byte-perfect, full ANSI colour on black, 19200
  baud.** Three 2.0-isms are fenced off for it: the build links
  `libnix13` (stock libnix implements the compiler's 32-bit
  multiply/divide helpers as utility.library calls, and utility.library
  does not exist before 2.0 -- without this the binary dies on startup
  complaining about it), the message ports and IORequests are
  hand-rolled statics (`CreateMsgPort`/`CreateIORequest` are V36+), and
  the screen open falls back from `OpenScreenTagList` to `OpenScreen`
  plus `LoadRGB4`/`ShowTitle` on a V34 intuition.

## Building

Requirements: the bebbo
[amiga-gcc](https://github.com/bebbo/amiga-gcc) toolchain
(`m68k-amigaos-gcc`, expected at `/opt/amiga`, override with `CC=`)
and `xdftool` from python
[amitools](https://github.com/cnvogelg/amitools). The source carries
its own minimal SDK slice (`amiga-mini.h`), so no NDK headers need to
be installed.

```sh
./build.sh          # produces retro32-term.adf
```

## Testing against a fake BBS (no network needed)

`fake-bbs-tcp.py` is a scripted stand-in BBS: it waits for a byte (the
visitor's Return), then sends a coloured ANSI banner and echoes
keystrokes. With a Copperline build that has `--serial-connect`
(v0.13+):

```sh
python3 fake-bbs-tcp.py &   # listens on 127.0.0.1:2323

printf '[floppy.df0]\npath = "retro32-term.adf"\n' > /tmp/term.toml
copperline KICK31.ROM --noaudio \
  --config /tmp/term.toml \
  --serial-connect 127.0.0.1:2323 \
  --press-after 20 enter \
  --screenshot-after 30 /tmp/term.png
```

(Note the disk must be configured as `df0` from power-on -- inserted
later, the boot strap has already passed it by.) Against the real
thing:

```sh
copperline KICK31.ROM --config /tmp/term.toml \
  --serial-connect bbs.retro32.com:1337
```

The desktop path is raw bytes with no telnet layer, so a few
negotiation bytes may print as noise at connect time; the Copperline
browser embed has a telnet layer and does not show this.

## No ZModem in v1

It is a terminal, not a transfer suite; if downloads become a request,
wiring in xprzmodem is the natural next step.

## License

Public domain, under [the Unlicense](LICENSE): use, modify, and
redistribute freely.
