# pc_side/pc_config.py
# ─────────────────────
# Edit these to match your setup before running task1_pc.py

RPI_IP   = "192.168.20.1"   # RPi's IP on your hotspot — check with: hostname -I on RPi
RPI_PORT = 5000              # must match RPI_PORT in RPi .env

MODEL_PATH = "weights/best.pt"  # path to your trained YOLO weights
