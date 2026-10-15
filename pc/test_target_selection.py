import unittest

from pc.image_recognition.target_selection import select_target_index


class TargetSelectionTests(unittest.TestCase):
    def test_larger_near_marker_beats_higher_confidence_background_marker(self):
        boxes = [
            [100, 200, 300, 500],
            [700, 250, 790, 430],
        ]
        confidences = [0.89, 0.98]

        self.assertEqual(select_target_index(boxes, confidences, 1000, 800), 0)

    def test_candidates_below_acceptance_threshold_are_rejected(self):
        boxes = [[100, 100, 400, 400]]
        self.assertIsNone(
            select_target_index(boxes, [0.537], 1000, 800, min_confidence=0.60)
        )

    def test_confidence_breaks_equal_area_tie(self):
        boxes = [[100, 100, 200, 200], [700, 100, 800, 200]]
        self.assertEqual(select_target_index(boxes, [0.70, 0.90], 1000, 800), 1)

    def test_centrality_breaks_equal_area_and_confidence_tie(self):
        boxes = [[50, 50, 150, 150], [450, 350, 550, 450]]
        self.assertEqual(select_target_index(boxes, [0.90, 0.90], 1000, 800), 1)

    def test_invalid_image_dimensions_return_none(self):
        self.assertIsNone(select_target_index([[0, 0, 1, 1]], [0.99], 0, 800))

    def test_ineligible_bullseye_is_not_selected_even_when_larger(self):
        boxes = [[100, 100, 900, 700], [420, 280, 620, 480]]
        confidences = [0.99, 0.82]
        self.assertEqual(
            select_target_index(
                boxes,
                confidences,
                1000,
                800,
                eligible=[False, True],
            ),
            1,
        )

    def test_only_ineligible_candidates_returns_none(self):
        boxes = [[100, 100, 900, 700]]
        self.assertIsNone(
            select_target_index(
                boxes,
                [0.99],
                1000,
                800,
                eligible=[False],
            )
        )


if __name__ == "__main__":
    unittest.main()
