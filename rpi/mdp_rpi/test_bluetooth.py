"""
test_bluetooth.py
─────────────────
Run this on the RPi to test Bluetooth connection with Android.
No STM, PC, or camera needed.

    python3 test_bluetooth.py

What it does:
1. Starts Bluetooth server and waits for Android to connect
2. Sends a welcome message to Android
3. Listens for any message from Android and echoes it back
4. Keeps running until you press Ctrl-C
"""

import logging
import time
from communications.android import Android

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [BT-TEST] %(levelname)s — %(message)s",
    datefmt="%H:%M:%S",
)

def main():
    android = Android()

    logging.info("=" * 50)
    logging.info("Bluetooth Test Starting")
    logging.info("=" * 50)

    android.start()
    logging.info("Bluetooth server started — open your Android app and connect now.")
    logging.info("Waiting for Android to connect...")

    # Wait until Android connects
    while not android.connected:
        time.sleep(0.5)

    logging.info("=" * 50)
    logging.info("Android connected successfully!")
    logging.info("=" * 50)

    # Send a test message to Android so you can verify RPi → Android works
    try:
        android.send("STATUS,RPi connected and working")
        logging.info("Sent welcome message to Android.")
    except Exception as e:
        logging.error(f"Failed to send welcome message: {e}")

    logging.info("Now listening for messages from Android...")
    logging.info("Send anything from your Android app — it will be echoed back.")
    logging.info("Press Ctrl-C to stop.")

    while True:
        try:
            # If Android disconnects, wait for reconnect
            if not android.connected:
                logging.warning("Android disconnected — waiting for reconnect...")
                while not android.connected:
                    time.sleep(0.5)
                logging.info("Android reconnected!")

            msg = android.receive()

            if msg:
                logging.info(f"RECEIVED FROM ANDROID: '{msg}'")

                # Echo it back so you can verify RPi → Android direction too
                try:
                    android.send(f"ECHO,{msg}")
                    logging.info(f"SENT BACK TO ANDROID: 'ECHO,{msg}'")
                except Exception as e:
                    logging.error(f"Failed to send echo: {e}")

            time.sleep(0.05)

        except KeyboardInterrupt:
            logging.info("Ctrl-C received — stopping.")
            break
        except Exception as e:
            logging.error(f"Unexpected error: {e}")
            time.sleep(1)

    android.stop()
    logging.info("Bluetooth test finished.")

if __name__ == "__main__":
    main()