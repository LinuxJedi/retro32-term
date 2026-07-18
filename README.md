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
  and renders through console.device on its own 8-colour hires screen.
  The ANSI palette is handed to intuition with SA_Colors -- the only
  ordering that reliably beats the OS default palette everywhere (AROS
  loads its preference colours onto every new screen) -- with
  SA_ShowTitle off so the screen bar stays behind the backdrop window.
  SGR colours 30-37 map one-to-one to pens; the Amiga console renders
  in Topaz 8 -- the font Amiga-oriented BBSes design for. Special keys
  (arrows) go out as ESC [ sequences.
- `s/startup-sequence` -- just runs `term`.

Incoming CSI sequences are reassembled whole and re-emitted in 8-bit
CSI form, normalized to exactly what the console is known to parse --
a whitelist, not a pass-through, because the AROS console's failure
mode for a sequence it cannot match is not "ignored": its command
scanner runs past the final byte and consumes the output that
follows, printing fragments of it as literal text (seen live as
mangled menu lines when the BBS sent its ECMA-48 font selection,
`ESC[0;40 D`). Concretely:

- Multi-parameter SGRs (`ESC[1;33m`) are split into consecutive
  single-parameter ones: the AROS console only parses the
  single-parameter form, and the split is semantically identical on
  Kickstart.
- Parameterized erase sequences are translated, because the Amiga
  console's `CSI J` / `CSI K` take no parameter and only erase toward
  the end (which left old text on screen at every BBS page change):
  `ESC[2J` becomes a formfeed (the console's whole-screen
  clear-and-home, matching the ANSI.SYS semantics BBSes assume),
  `ESC[2K` becomes CR plus erase-to-end-of-line, explicit `0`
  parameters are stripped, and the erase-backwards forms (`1J`/`1K`,
  no console equivalent, rare in BBS output) are dropped.
- `ESC[y;xf` (HVP) is rewritten to the `H` form the console
  understands, and `ESC[nG` (cursor to absolute column) becomes CR
  plus cursor-forward.
- Cursor motion, insert/delete, and scroll sequences pass through
  clamped to the parameter count the console accepts (one, two for
  `H`).
- Everything else -- private modes (`ESC[?25l`), intermediate bytes,
  colon subparameters, window ops, save/restore cursor -- is
  swallowed whole, exactly as a real terminal ignores what it does
  not implement.

The RBF interrupt handler copes with both interrupt-dispatch
conventions: classic exec calls it with RBF still pending (the handler
acks), while AROS acks INTREQ before dispatching (the word stays
latched in SERDATR). It therefore banks the first byte unconditionally
on entry -- the handler only ever runs because a word arrived. The
ring indices between the handler and the main loop are UWORDs: a 68000
writes a word atomically but a longword as two accesses, and a torn
32-bit counter silently drops bytes.

Line settings: 8N1, 19200 on both Kickstart and AROS. Paula buffers a
single received byte, so every byte must be serviced within one
character time; Kickstart manages 19200 comfortably. AROS's scheduler
masks all interrupts in roughly millisecond stretches -- longer than a
19200 character time -- which used to force 4800 there. The terminal
now sidesteps the AROS scheduler entirely: those masked stretches only
happen when the interrupt-exit path invokes the scheduler
(`core_ExitIntr` in AROS's `arch/m68k-all/kernel/kernel_intr.c`), and
that is skipped whenever task switching is disabled. So on AROS
(detected via `FindResident("aros.library")`) the main loop holds
`Forbid()` permanently and never calls `Wait()` -- it busy-polls
instead, which a dedicated kiosk disk can afford -- and the RBF
interrupt then always runs on time. Console output still works under a
permanent Forbid because AROS completes console `CMD_WRITE` in the
caller's context; console reads would not, so in this mode the keyboard
is polled straight from CIA-A (scancode from SDR, KDAT handshake by
beam-counter timing) and translated through console.device's
synchronous `RawKeyConvert()`. The trade-offs: on AROS no other task
ever runs again (fine, it is a kiosk; Ctrl-Amiga-Amiga still reboots)
and there is no key repeat (that was input.device's job). Change the
`BAUD_*` defines and rebuild to tune. The program also clears the
DMACON blitter-hog bit at startup, because console scrolls are big
blits that would otherwise lock the CPU off the chip bus for longer
than a character time.

Clearing blitter-hog is not enough on AROS, though: its m68k
`WaitBlit` (`arch/m68k-amiga/graphics/waitblit.S`) re-asserts
blitter-nasty for the duration of every wait and deliberately parks
the CPU off the chip bus until the blit completes. With no fast RAM,
the RBF handler's instruction fetches stall with it, so any console
blit longer than a character time silently overran Paula's one-byte
receive latch (measured: ~12 lost bytes per 10 KiB page, surfacing as
scattered mangled characters). The kiosk therefore `SetFunction()`s
graphics.library `WaitBlit` with a plain busy-spin that leaves
blitter-nasty alone: blits finish a little slower with the CPU
sharing the bus, and reception stays byte-perfect (verified with
emulator-side overrun instrumentation: zero drops across the same
replayed page).

The receive ring between the interrupt handler and the console is
32 KiB -- about 17 seconds of line-rate data. That is deliberate: the
AROS console renders SGR-heavy screens far slower than the 19200 line
rate, so most of a large BBS page sits in the ring while the console
catches up, and the ring must hold a whole page (the 8 KiB it replaced
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
- Kickstart 1.x: not supported (the console plumbing uses
  `CreateMsgPort`/`CreateIORequest`, V36+).

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
