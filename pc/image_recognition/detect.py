import logging
from pathlib import Path

import cv2
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "weights" / "best.pt"
RUNS_DIR = ROOT / "runs" / "predict"
RUNS_DIR.mkdir(parents=True, exist_ok=True)

model = YOLO(str(MODEL_PATH))

INFERENCE_SIZE = 640
YOLO_CONF = 0.25

def detect(image_path):
    """
    Detect the image target.

    Returns (class_name, confidence), or (None, None) when nothing is detected.
    The saved annotation draws only the same highest-confidence box that is
    returned to the RPi, so stitched output and TARGET messages stay aligned.
    """
    image = cv2.imread(str(image_path))
    if image is None:
        logging.error("Detector could not read image: %s", image_path)
        return None, None

    results = model.predict(
        source=image,
        imgsz=INFERENCE_SIZE,
        conf=YOLO_CONF,
        save=False,
        verbose=False,
    )

    result = results[0]
    boxes = result.boxes
    annotated_image = image.copy()

    if boxes is None or len(boxes) == 0:
        output_path = RUNS_DIR / Path(image_path).name
        saved = cv2.imwrite(str(output_path), annotated_image)
        if saved:
            logging.warning("Detector found no target; saved frame to %s", output_path)
        else:
            logging.error("Detector found no target and could not save %s", output_path)
        return None, None

    confidences = boxes.conf.cpu().numpy()
    best_index = int(confidences.argmax())
    class_id = int(boxes.cls[best_index].item())
    confidence = float(boxes.conf[best_index].item())
    class_name = result.names[class_id]

    x1, y1, x2, y2 = boxes.xyxy[best_index].cpu().numpy().astype(int)
    cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (255, 0, 255), 3)
    label = f"{class_name} {confidence:.2f}"
    cv2.putText(
        annotated_image,
        label,
        (x1, max(y1 - 10, 25)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 0, 255),
        2,
        cv2.LINE_AA,
    )

    output_path = RUNS_DIR / Path(image_path).name
    saved = cv2.imwrite(str(output_path), annotated_image)
    if saved:
        logging.info(
            "Detector found '%s' (confidence %.3f); saved annotation to %s",
            class_name,
            confidence,
            output_path,
        )
    else:
        logging.error(
            "Detector found '%s' (confidence %.3f) but could not save %s",
            class_name,
            confidence,
            output_path,
        )

    return class_name, confidence


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(levelname)s — %(message)s")
    test_image = input("Enter path to test image: ")
    class_name, confidence = detect(test_image)
    if class_name is None:
        print("Detected: NONE")
    else:
        print(f"Detected: {class_name} with confidence {confidence:.2f}")
