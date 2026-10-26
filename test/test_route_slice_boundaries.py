"""Regression tests for distance windows on synthetic reference paths."""

import unittest
from types import SimpleNamespace

import numpy as np

from commonroad_route_planner.frenet_tools.route_slice import RouteSlice


class TestRouteSliceBoundaries(unittest.TestCase):
    def setUp(self):
        self.points = np.column_stack((np.arange(11, dtype=float), np.zeros(11)))

    @staticmethod
    def make_route(points):
        # RouteSlice only needs these two geometry attributes from ReferencePath.
        return SimpleNamespace(
            reference_path=points,
            interpoint_distances=np.r_[
                0.0, np.linalg.norm(np.diff(points, axis=0), axis=1)
            ],
        )

    def assert_slice(self, query, ahead, behind, expected, points=None):
        points = self.points if points is None else points
        route = self.make_route(points)
        result = RouteSlice(
            route, *query, distance_ahead_in_m=ahead, distance_behind_in_m=behind
        )
        np.testing.assert_array_equal(result.reference_path, expected)
        self.assertIs(result.original_route, route)
        self.assertEqual(result.vehicle_point, list(query))
        return result

    def test_centered_window(self):
        self.assert_slice((5.0, 0.0), 3.0, 2.0, self.points[3:9])

    def test_first_vertex(self):
        self.assert_slice((0.0, 0.0), 4.0, 2.0, self.points[:5])

    def test_second_vertex_can_reach_start(self):
        self.assert_slice((1.0, 0.0), 3.0, 10.0, self.points[:5])

    def test_last_vertex_is_included(self):
        self.assert_slice((10.0, 0.0), 30.0, 4.0, self.points[6:])

    def test_penultimate_vertex_can_reach_end(self):
        self.assert_slice((9.0, 0.0), 2.0, 3.0, self.points[6:])

    def test_window_clamps_to_path_limits(self):
        self.assert_slice((5.0, 0.0), 100.0, 100.0, self.points)

    def test_nonuniform_spacing(self):
        points = np.column_stack(
            ([0.0, 1.0, 3.0, 6.0, 10.0, 15.0, 21.0, 28.0], np.zeros(8))
        )
        self.assert_slice((15.0, 0.0), 6.0, 5.0, points[4:7], points)

    def test_window_shorter_than_neighboring_segments(self):
        points = np.column_stack(([0.0, 1.0, 3.0, 6.0, 10.0, 15.0, 21.0], np.zeros(7)))
        self.assert_slice((10.0, 0.0), 2.0, 2.0, points[3:6], points)

    def test_query_snaps_to_nearest_vertex(self):
        self.assert_slice((5.2, 0.1), 3.0, 2.0, self.points[3:9])

    def test_zero_forward_distance(self):
        self.assert_slice((5.0, 0.0), 0.0, 3.0, self.points[2:6])

    def test_zero_backward_distance(self):
        self.assert_slice((5.0, 0.0), 3.0, 0.0, self.points[5:9])

    def test_distances_follow_curved_path(self):
        points = np.array([[0.0, 0.0], [3.0, 0.0], [3.0, 4.0], [6.0, 4.0], [6.0, 8.0]])
        self.assert_slice((3.0, 4.0), 4.0, 4.0, points[1:], points)

    def test_translated_reversed_path(self):
        points = self.points[::-1] + np.array([20.0, 7.0])
        self.assert_slice((25.0, 7.0), 3.0, 2.0, points[3:9], points)

    def test_original_geometry_is_unchanged(self):
        route = self.make_route(self.points)
        original_points = route.reference_path.copy()
        original_distances = route.interpoint_distances.copy()
        RouteSlice(route, 5.0, 0.0, 3.0, 2.0)
        np.testing.assert_array_equal(route.reference_path, original_points)
        np.testing.assert_array_equal(route.interpoint_distances, original_distances)


if __name__ == "__main__":
    unittest.main()
