import logging
import os
from pathlib import Path

import cv2
from ultralytics import YOLO

try:
    from .target_selection import select_target_index
except ImportError:  # task1_pc.py adds image_recognition/ directly to sys.path
    from target_selection import select_target_index


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "weights" / "best.pt"
RUNS_DIR = ROOT / "runs" / "predict"
RUNS_DIR.mkdir(parents=True, exist_ok=True)

model = YOLO(str(MODEL_PATH))

INFERENCE_SIZE = 640
YOLO_INFERENCE_CONFIDENCE = 0.25
YOLO_TARGET_CONFIDENCE = float(os.getenv("YOLO_TARGET_CONFIDENCE", "0.60"))
FALLBACK_CLASSES = {"bullseye"}

def detect(image_path):
    """
    Detect the image target.

    Returns (class_name, confidence), or (None, None) when nothing is detected.
    The saved annotation draws only the marker selected for the obstacle the
    robot is facing, so stitched output and TARGET messages stay aligned.
    """
    image = cv2.imread(str(image_path))
    if image is None:
        logging.error("Detector could not read image: %s", image_path)
        return None, None

    results = model.predict(
        source=image,
        imgsz=INFERENCE_SIZE,
        conf=YOLO_INFERENCE_CONFIDENCE,
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

    coordinates = boxes.xyxy.cpu().numpy()
    confidences = boxes.conf.cpu().numpy()
    class_names = [
        result.names[int(boxes.cls[index].item())]
        for index in range(len(boxes))
    ]
    preferred = [
        class_name.strip().lower() not in FALLBACK_CLASSES
        for class_name in class_names
    ]
    has_preferred_class = any(preferred)
    best_index = select_target_index(
        coordinates,
        confidences,
        image.shape[1],
        image.shape[0],
        min_confidence=YOLO_TARGET_CONFIDENCE,
        # Prefer every non-bullseye class. If the frame contains only
        # bullseyes, select and return the best bullseye as the final result.
        eligible=preferred if has_preferred_class else None,
    )
    if best_index is None:
        output_path = RUNS_DIR / Path(image_path).name
        saved = cv2.imwrite(str(output_path), annotated_image)
        logging.warning(
            "Detector candidates were all below target confidence %.2f; "
            "saved frame to %s%s",
            YOLO_TARGET_CONFIDENCE,
            output_path,
            "" if saved else " (save failed)",
        )
        return None, None

    accepted = [
        (class_names[i], float(confidences[i]))
        for i in range(len(boxes))
        if (preferred[i] or not has_preferred_class)
        and float(confidences[i]) >= YOLO_TARGET_CONFIDENCE
    ]
    if len(accepted) > 1:
        logging.info(
            "Detector saw multiple credible markers %s; selecting the largest apparent marker.",
            ", ".join(f"{name}:{confidence:.3f}" for name, confidence in accepted),
        )

    confidence = float(boxes.conf[best_index].item())
    class_name = class_names[best_index]

    x1, y1, x2, y2 = coordinates[best_index].astype(int)
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
