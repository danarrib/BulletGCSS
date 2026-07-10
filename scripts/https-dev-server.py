#!/usr/bin/env python3
"""Local HTTPS static server for UI/, for testing features that require a
secure context (e.g. Web Crypto) and don't work over plain HTTP.

Needs a TLS cert/key pair at scripts/certs/dev-cert.pem and dev-key.pem
(gitignored — generate your own per machine). mkcert is the easiest way:

    mkcert -install
    mkcert -cert-file scripts/certs/dev-cert.pem -key-file scripts/certs/dev-key.pem \\
        localhost 127.0.0.1 <your-lan-hostname> <your-lan-ip>

Include whatever hostname/IP you'll actually browse to from another device
on your LAN, so the cert's SAN list covers it. Override the cert location
with the DEV_CERT_DIR env var if you keep it elsewhere.
"""
import http.server
import ssl
import functools
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CERT_DIR = os.environ.get("DEV_CERT_DIR", os.path.join(SCRIPT_DIR, "certs"))
UI_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), "UI")
PORT = 8766

CERT_FILE = os.path.join(CERT_DIR, "dev-cert.pem")
KEY_FILE = os.path.join(CERT_DIR, "dev-key.pem")

if not (os.path.exists(CERT_FILE) and os.path.exists(KEY_FILE)):
    sys.exit(
        f"Missing dev certificate at {CERT_FILE} / {KEY_FILE}.\n"
        "Generate one with mkcert — see the docstring at the top of this file."
    )


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


handler = functools.partial(NoCacheHandler, directory=UI_DIR)
httpd = http.server.ThreadingHTTPServer(("0.0.0.0", PORT), handler)

ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
ctx.load_cert_chain(certfile=CERT_FILE, keyfile=KEY_FILE)
httpd.socket = ctx.wrap_socket(httpd.socket, server_side=True)

print(f"Serving {UI_DIR} on https://0.0.0.0:{PORT}")
httpd.serve_forever()
