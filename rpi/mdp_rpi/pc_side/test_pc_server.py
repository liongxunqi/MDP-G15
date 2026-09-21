import socket
import struct
import sys
import time
from pathlib import Path


# Keep this test server on port 5001, but share the exact detector used by
# the full Task 1 server. This prevents the two test paths from using different
# model paths, crops, class mappings, or selection rules.
PC_SIDE_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(PC_SIDE_DIR))
from image_recognition.detect import detect  # noqa: E402

HOST = "0.0.0.0"
PORT = 5001

SAVE_DIR = PC_SIDE_DIR / "received"
SAVE_DIR.mkdir(parents=True, exist_ok=True)


def recv_exact(conn, num_bytes):
    """
    Receive exactly num_bytes from the socket.
    Handles partial socket reads.
    """
    data = b""
    while len(data) < num_bytes:
        chunk = conn.recv(
            num_bytes - len(data)
        )
        if not chunk:
            raise ConnectionError(
                "Connection closed before all data received"
            )
        data += chunk
    return data


def handle_client(conn, addr):
    print(f"Connection from {addr}")

    # ── Receive image ────────────────────────────────────────────────
    size_bytes = recv_exact(
        conn,
        4
    )
    image_size = struct.unpack(
        ">I",
        size_bytes
    )[0]
    image_bytes = recv_exact(
        conn,
        image_size
    )

    # ── Save received image ──────────────────────────────────────────
    filename = SAVE_DIR / f"{time.time_ns()}.jpg"
    with filename.open("wb") as f:
        f.write(image_bytes)
    print(
        f"Saved image to {filename} "
        f"({image_size} bytes)"
    )

    # Use the same crop, model, confidence threshold, class mapping, and
    # annotation behavior as the full Task 1 detector.
    result_label, confidence = detect(str(filename))
    result_label = result_label or "none"
    print(
        f"Detected: {result_label}"
        + (f" ({confidence:.3f})" if confidence is not None else "")
    )
    # ── Send result to RPi ───────────────────────────────────────────
    conn.sendall(
        result_label.encode("utf-8")
    )


def main():
    with socket.socket(
        socket.AF_INET,
        socket.SOCK_STREAM
    ) as server:
        server.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )
        server.bind(
            (HOST, PORT)
        )
        server.listen(1)
        print(
            f"Listening on {HOST}:{PORT} ..."
        )

        while True:
            conn, addr = server.accept()
            with conn:
                try:
                    handle_client(
                        conn,
                        addr
                    )
                except Exception as e:
                    print(
                        f"Error handling client "
                        f"{addr}: {e}"
                    )


if __name__ == "__main__":
    main()
