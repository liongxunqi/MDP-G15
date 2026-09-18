import socket
import struct
import os
import time
import numpy as np
from ultralytics import YOLO

HOST = "0.0.0.0"   # listen on all network interfaces
PORT = 5001

SAVE_DIR = "received"
MODEL_PATH = "best.pt"

CLASS_NAMES = ['1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E',
            'F', 'G', 'H', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'bullseye',
            'dot', 'down_arrow', 'left_arrow', 'right_arrow', 'up_arrow']

os.makedirs(SAVE_DIR, exist_ok=True)
model = YOLO(MODEL_PATH)  # load once, reuse for every incoming image


def find_closest_bbox(results):
    bboxes = results[0].boxes.xyxy.cpu().numpy()
    class_ids = results[0].boxes.cls.cpu().numpy().astype(int)

    if len(bboxes) == 0:
        return "none"

    closest_idx = np.argmax(bboxes[:, 3] - bboxes[:, 1])
    return CLASS_NAMES[class_ids[closest_idx]]


def recv_exact(conn, num_bytes):
    """Receive exactly num_bytes from the socket (handles partial reads)."""
    data = b""
    while len(data) < num_bytes:
        chunk = conn.recv(num_bytes - len(data))
        if not chunk:
            raise ConnectionError("Connection closed before all data received")
        data += chunk
    return data


def handle_client(conn, addr):
    print(f"Connection from {addr}")

    # Read 4-byte length prefix, then that many bytes of image data
    size_bytes = recv_exact(conn, 4)
    image_size = struct.unpack(">I", size_bytes)[0]
    image_bytes = recv_exact(conn, image_size)

    filename = os.path.join(SAVE_DIR, f"{int(time.time())}.jpg")
    with open(filename, "wb") as f:
        f.write(image_bytes)
    print(f"Saved image to {filename} ({image_size} bytes)")

    results = model.predict(source=filename, save=True, project="runs", name="predict", exist_ok=True)
    result_label = find_closest_bbox(results)
    print(f"Detected: {result_label}")

    conn.sendall(result_label.encode("utf-8"))


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen(1)
        print(f"Listening on {HOST}:{PORT} ...")

        while True:
            conn, addr = server.accept()
            with conn:
                try:
                    handle_client(conn, addr)
                except Exception as e:
                    print(f"Error handling client {addr}: {e}")


if __name__ == "__main__":
    main()