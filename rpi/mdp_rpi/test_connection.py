"""
test_connection.py  —  Smoke test for RPi ↔ PC socket connection
─────────────────────────────────────────────────────────────────
Run this INSTEAD of task1.py to verify the TCP link works without needing
STM, Android, or a camera.

On RPi:
    python3 test_connection.py --mode server

On PC (in a separate terminal):
    python3 test_connection.py --mode client --ip <RPi IP>

Expected output:
    Server side:  receives "HELLO FROM PC", replies "HELLO FROM RPI"
    Client side:  receives "HELLO FROM RPI"
"""

import argparse
import socket
import logging

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")

RPI_PORT = 5000


def run_server(host: str = "0.0.0.0", port: int = RPI_PORT) -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    logging.info(f"Server listening on {host}:{port} — waiting for PC…")
    conn, addr = srv.accept()
    logging.info(f"PC connected from {addr}.")

    data = conn.recv(1024).decode("utf-8")
    logging.info(f"Received: '{data.strip()}'")
    conn.sendall("HELLO FROM RPI\n".encode("utf-8"))
    logging.info("Replied: HELLO FROM RPI")

    conn.close()
    srv.close()
    logging.info("Test passed ✓")


def run_client(ip: str, port: int = RPI_PORT) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    logging.info(f"Connecting to {ip}:{port}…")
    sock.connect((ip, port))
    logging.info("Connected.")
    sock.sendall("HELLO FROM PC\n".encode("utf-8"))
    logging.info("Sent: HELLO FROM PC")
    reply = sock.recv(1024).decode("utf-8")
    logging.info(f"Received: '{reply.strip()}'")
    sock.close()
    logging.info("Test passed ✓")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["server", "client"], required=True)
    parser.add_argument("--ip", default="192.168.20.1", help="RPi IP (client mode only)")
    parser.add_argument("--port", type=int, default=RPI_PORT)
    args = parser.parse_args()

    if args.mode == "server":
        run_server(port=args.port)
    else:
        run_client(args.ip, args.port)
