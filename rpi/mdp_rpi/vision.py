"""
vision.py  —  on-RPi image recognition for checklist A.5
─────────────────────────────────────────────────────────
Task 1 sends JPEGs to the PC and lets YOLO run there. A.5 does not need the
PC at all — it is one obstacle and a handful of stills — so this module runs
the same weights locally and keeps the whole checklist item on the Pi.

The import of ultralytics is deliberately LAZY. It drags in torch, which is a
slow, heavy import on a Pi and is not in requirements_rpi.txt. Nothing should
pay that cost just for importing this file, and a missing install should
produce one clear sentence rather than a traceback out of a library.

  pip install ultralytics        # ~1-2 GB with torch, takes a while on a Pi

Weights default to the ones already in the repo (pc_side/weights/best.pt).
Override with A5_WEIGHTS in .env if yours live elsewhere.
"""

import logging
import os
from typing import Optional, Tuple

# Same list, same order as pc_side/image_recognition/detect.py. The model
# outputs indices into this list, so the ORDER is part of the trained model —
# do not sort it, do not add to the middle.
CLASS_NAMES = ['1', '2', '3', '4', '5', '6', '7', '8', '9',
               'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'S',
               'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
               'bullseye', 'dot', 'down_arrow', 'left_arrow',
               'right_arrow', 'up_arrow']

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_WEIGHTS = os.path.join(_HERE, "pc_side", "weights", "best.pt")

_model = None   # loaded once, on first detect()


def _load_model():
    """Load YOLO once and keep it. First call is the slow one."""
    global _model
    if _model is not None:
        return _model

    weights = os.getenv("A5_WEIGHTS", _DEFAULT_WEIGHTS)
    if not os.path.exists(weights):
        raise FileNotFoundError(
            f"YOLO weights not found at '{weights}'. Put best.pt there, or set "
            f"A5_WEIGHTS in .env to wherever it actually lives."
        )

    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise ImportError(
            "ultralytics is not installed on this Pi, so A.5 cannot run "
            "detection locally. Install it with 'pip install ultralytics' "
            "(large — it pulls in torch), or run detection on the PC instead."
        ) from exc

    logging.info(f"Vision: loading weights from {weights} (first load is slow)…")
    _model = YOLO(weights)
    logging.info("Vision: model ready.")
    return _model


def detect(image_path: str) -> Tuple[Optional[str], Optional[float]]:
    """
    Run YOLO on one still.

    Returns (class_name, confidence), or (None, None) if the model found
    nothing at all. Picks the detection with the TALLEST bounding box, which
    on an arena image is the nearest symbol — the same rule the PC side uses,
    kept identical so results do not change depending on where you ran it.

    Annotated copies are written to runs/predict/ for the demo and the report.
    """
    model = _load_model()

    results = model.predict(
        source=image_path,
        save=True,
        project=os.path.join(_HERE, "runs"),
        name="predict",
        exist_ok=True,
        verbose=False,
    )

    boxes = results[0].boxes
    if boxes is None or len(boxes.xyxy) == 0:
        return None, None

    xyxy = boxes.xyxy.cpu().numpy()
    class_ids = boxes.cls.cpu().numpy().astype(int)
    confs = boxes.conf.cpu().numpy()

    # Tallest box == nearest symbol.
    idx = int((xyxy[:, 3] - xyxy[:, 1]).argmax())

    cls = int(class_ids[idx])
    if not (0 <= cls < len(CLASS_NAMES)):
        logging.warning(f"Vision: model returned class index {cls}, off the end "
                        f"of CLASS_NAMES ({len(CLASS_NAMES)}). Weights and class "
                        f"list disagree.")
        return None, None

    return CLASS_NAMES[cls], float(confs[idx])
