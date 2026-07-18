/*
 * amiga-mini.h - the slice of the AmigaOS SDK retro32term.c needs, self
 * contained so the terminal builds with a bare m68k-amigaos-gcc even when
 * no (or a broken) NDK header set is installed. Struct layouts follow the
 * published AmigaOS ABI (RKM/NDK 3.x, 2-byte alignment, which is the m68k
 * ABI this compiler targets); library calls go through the NDK's standard
 * LPx inline macros (<inline/macros.h>, shipped with the bebbo toolchain),
 * with LVO offsets taken from the matching inline headers.
 */

#ifndef AMIGA_MINI_H
#define AMIGA_MINI_H

/* --- basic types ------------------------------------------------------- */

typedef unsigned char UBYTE;
typedef signed char BYTE;
typedef unsigned short UWORD;
typedef signed short WORD;
typedef unsigned long ULONG;
typedef signed long LONG;
typedef void *APTR;
typedef short BOOL;
typedef const char *CONST_STRPTR;

#define TRUE 1
#define FALSE 0
#ifndef NULL
#define NULL ((void *)0)
#endif

/* --- exec structures ---------------------------------------------------- */

struct Node {
    struct Node *ln_Succ;
    struct Node *ln_Pred;
    UBYTE ln_Type;
    BYTE ln_Pri;
    char *ln_Name;
};

struct List {
    struct Node *lh_Head;
    struct Node *lh_Tail;
    struct Node *lh_TailPred;
    UBYTE lh_Type;
    UBYTE l_pad;
};

struct MsgPort {
    struct Node mp_Node;
    UBYTE mp_Flags;
    UBYTE mp_SigBit; /* offset 15 */
    APTR mp_SigTask;
    struct List mp_MsgList;
};

struct Message {
    struct Node mn_Node;
    struct MsgPort *mn_ReplyPort;
    UWORD mn_Length;
};

struct IORequest {
    struct Message io_Message;
    APTR io_Device;
    APTR io_Unit;
    UWORD io_Command;
    UBYTE io_Flags;
    BYTE io_Error;
};

struct IOStdReq {
    struct Message io_Message;
    APTR io_Device;
    APTR io_Unit;
    UWORD io_Command;
    UBYTE io_Flags;
    BYTE io_Error;
    ULONG io_Actual;
    ULONG io_Length;
    APTR io_Data;
    ULONG io_Offset;
};

/* devices/serial.h */
struct IOTArray {
    ULONG TermArray0;
    ULONG TermArray1;
};

struct IOExtSer {
    struct IOStdReq IOSer;
    ULONG io_CtlChar;
    ULONG io_RBufLen;
    ULONG io_ExtFlags;
    ULONG io_Baud;
    ULONG io_BrkTime;
    struct IOTArray io_TermArray;
    UBYTE io_ReadLen;
    UBYTE io_WriteLen;
    UBYTE io_StopBits;
    UBYTE io_SerFlags;
    UWORD io_Status;
};

struct Task; /* opaque */
struct Resident; /* opaque */

struct Interrupt {
    struct Node is_Node;
    APTR is_Data;
    void (*is_Code)(void);
};

#define NT_INTERRUPT 2

#define CMD_READ 2
#define CMD_WRITE 3
#define SDCMD_QUERY 9
#define SDCMD_SETPARAMS 11
#define SERF_XDISABLED (1 << 7)
#define SERF_SHARED (1 << 5)

#define CONU_STANDARD 0

#define SIGBREAKF_CTRL_C (1UL << 12)

/* --- Paula serial hardware ----------------------------------------------- */

#define CUSTOM_SERDATR (*(volatile UWORD *)0xDFF018)
#define CUSTOM_SERDAT (*(volatile UWORD *)0xDFF030)
#define CUSTOM_SERPER (*(volatile UWORD *)0xDFF032)
#define CUSTOM_INTENA (*(volatile UWORD *)0xDFF09A)
#define CUSTOM_INTREQ (*(volatile UWORD *)0xDFF09C)
#define CUSTOM_DMACON (*(volatile UWORD *)0xDFF096)

#define DMAF_BLITHOG 0x0400

#define SERDATR_OVRUN (1 << 15) /* receive overrun; cleared by RBF ack */
#define SERDATR_RBF (1 << 14)   /* receive buffer full */
#define SERDATR_TBE (1 << 13)   /* transmit buffer empty */

#define INTB_RBF 11
#define INTF_SETCLR 0x8000
#define INTF_INTEN 0x4000
#define INTF_RBF 0x0800
#define INTF_EXTER 0x2000
#define INTF_PORTS 0x0008

/* VHPOSR: vertical beam position (low 8 bits) in the high byte. One
 * raster line is 63.5 us; counting its transitions gives a delay that
 * needs no timer hardware. */
#define CUSTOM_VHPOSR (*(volatile UWORD *)0xDFF006)

/* --- CIA-A (keyboard) ---------------------------------------------------- */

#define CIAA_SDR (*(volatile UBYTE *)0xBFEC01) /* keyboard shift register */
#define CIAA_ICR (*(volatile UBYTE *)0xBFED01) /* int flags; read clears */
#define CIAA_CRA (*(volatile UBYTE *)0xBFEE01)

#define CIAICRF_SP 0x08     /* serial-port (keyboard byte) int flag */
#define CIACRAF_SPMODE 0x40 /* SP line output mode: drives KDAT low */

/* --- input events (devices/inputevent.h) --------------------------------- */

#define IECLASS_RAWKEY 0x01
#define IECODE_UP_PREFIX 0x80

struct InputEvent {
    struct InputEvent *ie_NextEvent;
    UBYTE ie_Class;
    UBYTE ie_SubClass;
    UWORD ie_Code;
    UWORD ie_Qualifier;
    ULONG ie_EventAddress; /* union ie_position; zeroed here */
    ULONG ie_Seconds;      /* struct timeval ie_TimeStamp */
    ULONG ie_Micros;
};

struct KeyMap; /* opaque; passing NULL selects the default keymap */

/* --- utility tags --------------------------------------------------------- */

struct TagItem {
    ULONG ti_Tag;
    ULONG ti_Data;
};

#define TAG_DONE 0UL
#define TAG_USER 0x80000000UL

/* --- intuition / graphics ------------------------------------------------ */

struct Library;
struct Screen;  /* opaque */
struct Window;  /* opaque; console.device only needs the pointer */
struct ViewPort;

/* intuition/intuition.h: SA_Colors entry; 4-bit gun values, terminated by
 * ColorIndex = -1. */
struct ColorSpec {
    WORD ColorIndex;
    UWORD Red;
    UWORD Green;
    UWORD Blue;
};

/* graphics/text.h: font request, for NewScreen.Font. */
struct TextAttr {
    const char *ta_Name;
    UWORD ta_YSize;
    UBYTE ta_Style;
    UBYTE ta_Flags;
};

/* intuition/screens.h screen tags (V36+). */
#define SA_Dummy (TAG_USER + 32)
#define SA_Width (SA_Dummy + 3)
#define SA_Height (SA_Dummy + 4)
#define SA_Depth (SA_Dummy + 5)
#define SA_Colors (SA_Dummy + 9)
#define SA_Type (SA_Dummy + 13)
#define SA_ShowTitle (SA_Dummy + 22)
#define SA_Quiet (SA_Dummy + 24)

struct NewScreen {
    WORD LeftEdge, TopEdge, Width, Height, Depth;
    UBYTE DetailPen, BlockPen;
    UWORD ViewModes;
    UWORD Type;
    APTR Font;
    UBYTE *DefaultTitle;
    APTR Gadgets;
    APTR CustomBitMap;
};

struct NewWindow {
    WORD LeftEdge, TopEdge, Width, Height;
    UBYTE DetailPen, BlockPen;
    ULONG IDCMPFlags;
    ULONG Flags;
    APTR FirstGadget;
    APTR CheckMark;
    UBYTE *Title;
    struct Screen *Screen;
    APTR BitMap;
    WORD MinWidth, MinHeight;
    UWORD MaxWidth, MaxHeight;
    UWORD Type;
};

/* sizeof(struct Window) for console.device's io_Length; the device only
 * dereferences io_Data. */
#define WINDOW_SIZEOF 136

#define HIRES 0x8000
#define CUSTOMSCREEN 0x000F
#define SCREENQUIET 0x0100

#define WFLG_SIMPLE_REFRESH 0x00000040UL
#define WFLG_BACKDROP 0x00000100UL
#define WFLG_BORDERLESS 0x00000800UL
#define WFLG_ACTIVATE 0x00001000UL
#define WFLG_RMBTRAP 0x00010000UL

/* --- library bases and calls --------------------------------------------- */

#define SysBase (*(struct Library **)4UL)
extern struct Library *IntuitionBase;
extern struct Library *GfxBase;

/* <inline/stubs.h> only pulls NDK headers and forward-declares structs;
 * everything this program needs is declared above, so satisfy its guard
 * instead of including it. */
#define __INLINE_STUB_H
#include <inline/macros.h>

/* exec.library (offsets from inline/exec.h) */
#define Forbid() \
    LP0NR(0x84, Forbid, , SysBase)
#define Permit() \
    LP0NR(0x8a, Permit, , SysBase)
#define Wait(sigs) \
    LP1(0x13e, ULONG, Wait, ULONG, sigs, d0, , SysBase)
#define GetMsg(port) \
    LP1(0x174, struct Message *, GetMsg, struct MsgPort *, port, a0, , SysBase)
#define OpenLibrary(name, ver) \
    LP2(0x228, struct Library *, OpenLibrary, CONST_STRPTR, name, a1, ULONG, ver, d0, , SysBase)
#define CloseLibrary(lib) \
    LP1NR(0x19e, CloseLibrary, struct Library *, lib, a1, , SysBase)
#define OpenDevice(name, unit, req, flags) \
    LP4(0x1bc, BYTE, OpenDevice, CONST_STRPTR, name, a0, ULONG, unit, d0, \
        struct IORequest *, req, a1, ULONG, flags, d1, , SysBase)
#define CloseDevice(req) \
    LP1NR(0x1c2, CloseDevice, struct IORequest *, req, a1, , SysBase)
#define DoIO(req) \
    LP1(0x1c8, BYTE, DoIO, struct IORequest *, req, a1, , SysBase)
#define SendIO(req) \
    LP1NR(0x1ce, SendIO, struct IORequest *, req, a1, , SysBase)
#define CheckIO(req) \
    LP1(0x1d4, struct IORequest *, CheckIO, struct IORequest *, req, a1, , SysBase)
#define WaitIO(req) \
    LP1(0x1da, BYTE, WaitIO, struct IORequest *, req, a1, , SysBase)
#define AbortIO(req) \
    LP1NR(0x1e0, AbortIO, struct IORequest *, req, a1, , SysBase)
#define CreateIORequest(port, size) \
    LP2(0x28e, APTR, CreateIORequest, struct MsgPort *, port, a0, ULONG, size, d0, , SysBase)
#define DeleteIORequest(req) \
    LP1NR(0x294, DeleteIORequest, APTR, req, a0, , SysBase)
#define CreateMsgPort() \
    LP0(0x29a, struct MsgPort *, CreateMsgPort, , SysBase)
#define FindName(list, name) \
    LP2(0x114, struct Node *, FindName, struct List *, list, a0, CONST_STRPTR, name, a1, , SysBase)
#define FindResident(name) \
    LP1(0x60, struct Resident *, FindResident, CONST_STRPTR, name, a1, , SysBase)
#define DeleteMsgPort(port) \
    LP1NR(0x2a0, DeleteMsgPort, struct MsgPort *, port, a0, , SysBase)
#define SetIntVector(num, interrupt) \
    LP2(0xa2, struct Interrupt *, SetIntVector, LONG, num, d0, struct Interrupt *, interrupt, a1, , SysBase)
#define SetFunction(lib, offset, func) \
    LP3(0x1a4, APTR, SetFunction, struct Library *, lib, a1, LONG, offset, a0, \
        APTR, func, d0, , SysBase)
#define FindTask(name) \
    LP1(0x126, struct Task *, FindTask, CONST_STRPTR, name, a1, , SysBase)
#define Signal(task, sigs) \
    LP2NR(0x144, Signal, struct Task *, task, a1, ULONG, sigs, d0, , SysBase)
#define AllocSignal(num) \
    LP1(0x14a, BYTE, AllocSignal, LONG, num, d0, , SysBase)
#define FreeSignal(num) \
    LP1NR(0x150, FreeSignal, LONG, num, d0, , SysBase)

/* intuition.library (offsets from inline/intuition.h) */
#define OpenScreen(ns) \
    LP1(0xc6, struct Screen *, OpenScreen, struct NewScreen *, ns, a0, , IntuitionBase)
#define OpenScreenTagList(ns, tags) \
    LP2(0x264, struct Screen *, OpenScreenTagList, struct NewScreen *, ns, a0, \
        struct TagItem *, tags, a1, , IntuitionBase)
#define CloseScreen(s) \
    LP1(0x42, BOOL, CloseScreen, struct Screen *, s, a0, , IntuitionBase)
#define OpenWindow(nw) \
    LP1(0xcc, struct Window *, OpenWindow, struct NewWindow *, nw, a0, , IntuitionBase)
#define CloseWindow(w) \
    LP1NR(0x48, CloseWindow, struct Window *, w, a0, , IntuitionBase)
#define ViewPortAddress(w) \
    LP1(0x12c, struct ViewPort *, ViewPortAddress, struct Window *, w, a0, , IntuitionBase)

/* graphics.library (offset from inline/graphics.h) */
#define SetRGB4(vp, n, r, g, b) \
    LP5NR(0x120, SetRGB4, struct ViewPort *, vp, a0, WORD, n, d0, \
          UBYTE, r, d1, UBYTE, g, d2, UBYTE, b, d3, , GfxBase)

/* console.device library-style call (offset from inline/console.h; the
 * base is the opened device, io_Device from any console IORequest).
 * Translates one IECLASS_RAWKEY InputEvent through the keymap without
 * involving any other task, so it is callable under Forbid(). */
extern struct Library *ConsoleDevice;
#define RawKeyConvert(events, buffer, length, keyMap) \
    LP4(0x30, LONG, RawKeyConvert, struct InputEvent *, events, a0, \
        UBYTE *, buffer, a1, LONG, length, d1, \
        struct KeyMap *, keyMap, a2, , ConsoleDevice)

#endif /* AMIGA_MINI_H */
