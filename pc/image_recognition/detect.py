import logging
import os
from pathlib import Path

import cv2
from ultralytics import YOLO

try:
    from .target_selection import select_target_index, select_unique_arrow_index
except ImportError:  # task1_pc.py adds image_recognition/ directly to sys.path
    from target_selection import select_target_index, select_unique_arrow_index


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "weights" / "best.pt"
RUNS_DIR = ROOT / "runs" / "predict"
RUNS_DIR.mkdir(parents=True, exist_ok=True)

model = YOLO(str(MODEL_PATH))

INFERENCE_SIZE = 640
YOLO_INFERENCE_CONFIDENCE = 0.25
YOLO_TARGET_CONFIDENCE = float(os.getenv("YOLO_TARGET_CONFIDENCE", "0.60"))
FALLBACK_CLASSES = {"bullseye"}

def detect(image_path, allowed_classes=None, output_dir=None,
           selection_policy="nearest", target_confidence=None):
    """
    Detect the image target.

    Returns (class_name, confidence), or (None, None) when nothing is detected.
    The saved annotation draws only the marker selected for the obstacle the
    robot is facing, so stitched output and TARGET messages stay aligned.
    """
    if selection_policy not in ("nearest", "unique_arrow"):
        raise ValueError("Unknown target selection policy")
    threshold = YOLO_TARGET_CONFIDENCE if target_confidence is None else float(target_confidence)
    if not 0 <= threshold <= 1:
        raise ValueError("Target confidence must be between zero and one")
    destination = Path(output_dir) if output_dir is not None else RUNS_DIR
    destination.mkdir(parents=True, exist_ok=True)
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
        output_path = destination / Path(image_path).name
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
    if allowed_classes is not None:
        allowed = {name.strip().lower() for name in allowed_classes}
        preferred = [name.strip().lower() in allowed for name in class_names]
    has_preferred_class = any(preferred)
    selector = select_unique_arrow_index if selection_policy == "unique_arrow" else select_target_index
    best_index = selector(
        coordinates,
        confidences,
        image.shape[1],
        image.shape[0],
        min_confidence=threshold,
        # Prefer every non-bullseye class. If the frame contains only
        # bullseyes, select and return the best bullseye as the final result.
        eligible=preferred if has_preferred_class or allowed_classes is not None else None,
    )
    if best_index is None:
        output_path = destination / Path(image_path).name
        saved = cv2.imwrite(str(output_path), annotated_image)
        logging.warning(
            "Detector has no unambiguous eligible target (policy %s, confidence %.2f); "
            "saved frame to %s%s",
            selection_policy, threshold,
            output_path,
            "" if saved else " (save failed)",
        )
        return None, None

    accepted = [
        (class_names[i], float(confidences[i]))
        for i in range(len(boxes))
        if (preferred[i] or not has_preferred_class)
        and float(confidences[i]) >= threshold
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

    output_path = destination / Path(image_path).name
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
