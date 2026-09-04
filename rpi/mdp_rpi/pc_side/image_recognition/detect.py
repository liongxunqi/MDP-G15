from ultralytics import YOLO
import numpy as np
import os

CLASS_NAMES = ['1', '2', '3', '4', '5', '6', '7', '8', '9',
            'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'S',
            'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
            'bullseye', 'dot', 'down_arrow', 'left_arrow',
            'right_arrow', 'up_arrow']

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
model = YOLO(os.path.join(ROOT, "image_detection", "best.pt"))


def find_closest_bbox(results):
    bboxes = results[0].boxes.xyxy.cpu().numpy()
    class_ids = results[0].boxes.cls.cpu().numpy().astype(int)
    confs = results[0].boxes.conf.cpu().numpy()

    if len(bboxes) == 0:
        return None, None

    closest_idx = np.argmax(bboxes[:, 3] - bboxes[:, 1])
    return CLASS_NAMES[class_ids[closest_idx]], float(confs[closest_idx])


def detect(image_path):
    results = model.predict(
        source=image_path,
        save=True,
        project=os.path.join(ROOT, "runs"),
        name="predict",
        exist_ok=True
    )
    class_name, confidence = find_closest_bbox(results)
    return class_name, confidence


if __name__ == "__main__":
    test_image = input("Enter path to test image: ")
    class_name, confidence = detect(test_image)
    print(f"Detected: {class_name} with confidence {confidence:.2f}")