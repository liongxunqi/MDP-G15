from pathlib import Path

import cv2
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "weights" / "best.pt"
RUNS_DIR = ROOT / "runs" / "predict"
RUNS_DIR.mkdir(parents=True, exist_ok=True)

model = YOLO(str(MODEL_PATH))

INFERENCE_SIZE = 960
YOLO_CONF = 0.25


def centre_crop(image):
    """Keep the central region where the robot-facing target should appear."""
    height, width = image.shape[:2]
    crop_width = int(width * 0.70)
    crop_height = int(height * 0.80)
    x1 = (width - crop_width) // 2
    y1 = (height - crop_height) // 2
    return image[y1:y1 + crop_height, x1:x1 + crop_width]


def detect(image_path):
    """
    Detect the image target.

    Returns (class_name, confidence), or (None, None) when nothing is detected.
    The saved annotation draws only the same highest-confidence box that is
    returned to the RPi, so stitched output and TARGET messages stay aligned.
    """
    image = cv2.imread(str(image_path))
    if image is None:
        print(f"Could not read image: {image_path}")
        return None, None

    cropped = centre_crop(image)
    results = model.predict(
        source=cropped,
        imgsz=INFERENCE_SIZE,
        conf=YOLO_CONF,
        save=False,
        verbose=False,
    )

    result = results[0]
    boxes = result.boxes
    annotated_image = cropped.copy()

    if boxes is None or len(boxes) == 0:
        output_path = RUNS_DIR / Path(image_path).name
        saved = cv2.imwrite(str(output_path), annotated_image)
        print("\n──── Detection Result ────")
        print("No detection")
        if saved:
            print(f"Saved: {output_path}")
        else:
            print(f"Could not save: {output_path}")
        print("──────────────────────────\n")
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

    print("\n──── Detection Result ────")
    print(f"Class: {class_name}")
    print(f"Confidence: {confidence:.3f}")
    print(f"Saved: {output_path}" if saved else f"Could not save: {output_path}")
    print("──────────────────────────\n")

    return class_name, confidence


if __name__ == "__main__":
    test_image = input("Enter path to test image: ")
    class_name, confidence = detect(test_image)
    if class_name is None:
        print("Detected: NONE")
    else:
        print(f"Detected: {class_name} with confidence {confidence:.2f}")
