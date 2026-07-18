#!/usr/bin/env python3
"""Plain-TCP fake BBS for testing the terminal disk with --serial-connect.

Waits for the first byte from the terminal (the visitor pressing Return),
then sends a coloured ANSI banner and echoes everything back with a
prefix. Received bytes are logged to stdout as hex. Listens on
127.0.0.1:2323 (override with argv[1] as PORT).
"""
import socket
import sys

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 2323

BANNER = (
    b"\x1b[2J\x1b[H"
    b"\x1b[1;33m*** FakeBBS ***\x1b[0m\r\n"
    b"\x1b[31mred \x1b[32mgreen \x1b[34mblue \x1b[36mcyan "
    b"\x1b[35mmagenta \x1b[37mwhite\x1b[0m\r\n"
    b"Type and I echo.\r\n"
)

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("127.0.0.1", PORT))
srv.listen(1)
print(f"listening on 127.0.0.1:{PORT}", flush=True)
while True:
    conn, peer = srv.accept()
    print(f"connect from {peer}", flush=True)
    sent_banner = False
    try:
        while True:
            data = conn.recv(4096)
            if not data:
                break
            print("rx", data.hex(), flush=True)
            if not sent_banner:
                conn.sendall(BANNER)
                sent_banner = True
            conn.sendall(b"echo: " + data + b"\r\n")
    except OSError:
        pass
    print("disconnect", flush=True)
    conn.close()
