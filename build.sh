#!/bin/sh -e
# Build retro32-term.adf: compile the terminal with the bebbo m68k-amigaos
# toolchain and assemble a bootable OFS floppy with xdftool (amitools).
#
#   CC=...  override the compiler (default /opt/amiga/bin/m68k-amigaos-gcc)
#
# Output: retro32-term.adf next to this script.

cd "$(dirname "$0")"
CC="${CC:-/opt/amiga/bin/m68k-amigaos-gcc}"

"$CC" -O2 -fomit-frame-pointer -noixemul -o term retro32term.c

rm -f retro32-term.adf
xdftool retro32-term.adf create + format "Retro32" ofs + boot install \
  + write term \
  + makedir s \
  + write startup-sequence s/startup-sequence

xdftool retro32-term.adf list
echo "OK: retro32-term.adf"
