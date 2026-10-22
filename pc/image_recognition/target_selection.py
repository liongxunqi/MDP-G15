"""Choose the marker belonging to the obstacle nearest the camera."""


def select_target_index(boxes, confidences, image_width, image_height,
                        min_confidence=0.60, eligible=None):
    """Return the best box index, or None when no candidate is trustworthy.

    Task 1 markers have the same physical dimensions. When multiple obstacles
    appear in one frame, the marker on the obstacle the robot is facing should
    therefore have the largest image area. Confidence and centrality are only
    tie-breakers; using confidence first can select a smaller background board.
    ``eligible`` lets the detector perform a preferred-class pass without
    changing box indices; it can run an unrestricted fallback when needed.
    """
    if image_width <= 0 or image_height <= 0:
        return None

    best_index = None
    best_key = None
    image_area = float(image_width * image_height)

    if eligible is None:
        eligible = [True] * len(boxes)

    for index, (box, confidence, is_eligible) in enumerate(
            zip(boxes, confidences, eligible)):
        if not is_eligible:
            continue
        confidence = float(confidence)
        if confidence < min_confidence:
            continue

        x1, y1, x2, y2 = (float(value) for value in box)
        width = max(0.0, x2 - x1)
        height = max(0.0, y2 - y1)
        area = width * height / image_area
        centre_x = (x1 + x2) / (2.0 * image_width)
        centre_y = (y1 + y2) / (2.0 * image_height)
        centre_distance_sq = (centre_x - 0.5) ** 2 + (centre_y - 0.5) ** 2

        key = (area, confidence, -centre_distance_sq)
        if best_key is None or key > best_key:
            best_index = index
            best_key = key

    return best_index
