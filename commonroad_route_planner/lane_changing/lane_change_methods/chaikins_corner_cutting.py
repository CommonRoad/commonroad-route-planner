"""
def asdf(self):

    reference_path: np.ndarray = None
    skip_ids: List[int] = list()

    for idx, id_lanelet in enumerate(self._lanelet_ids):


        # Sample the center vertices of the lanelet as foundation for the reference path
        lanelet: "Lanelet" = self._lanelet_network.find_lanelet_by_id(id_lanelet)
        centerline_vertices: np.ndarray = pops.sample_polyline(lanelet.center_vertices, step_resample)
        num_vertices: int = len(centerline_vertices)

        # get driving instruction object for lanelet
        instruction: LaneChangeInstruction = self._lane_change_position_handler.get_driving_instruction_for_lanelet(lanelet)




        # FIXME: Does not sound very practical
        # Number of vertices to be used in the lane change
        num_vertices_lane_change: int = min(
            int(num_vertices * percentage_vertices_lane_change_max) + 1,
            num_vertices_lane_change_max,
        )


        # First time computation at initial lanelet
        if(reference_path is None):
            idx_start = int(instruction.lanelet_portions[0] * num_vertices)
            idx_end = int(instruction.lanelet_portions[1] * num_vertices)

            # reserve some vertices if it is not the last lanelet
            if idx != (num_lanelets_in_route - 1):
                idx_end = idx_end - num_vertices_lane_change


            # Since we are rounding down, make sure that idx_end is not zero
            idx_end = max(idx_end, 1)
            reference_path: np.ndarray = centerline_vertices[idx_start:idx_end, :]


        # Concatenate new parts to old reference path
        else:
            idx_start = (
                int(instruction.lanelet_portions[0] * num_vertices) + num_vertices_lane_change
            )
            # prevent index out of bound, since we are alway rounding down
            idx_start = min(idx_start, num_vertices - 1)

            idx_end = int(instruction.lanelet_portions[1] * num_vertices)
            # reserve some vertices if it is not the last lanelet
            if idx != (num_lanelets_in_route - 1):
                idx_end = idx_end - num_vertices_lane_change
                # prevent index out of bound
                idx_end = max(idx_end, 1)

            path_to_be_concatenated: np.ndarray = centerline_vertices[idx_start:idx_end, :]

            reference_path: np.ndarray = np.concatenate(
                (reference_path, path_to_be_concatenated), axis=0
            )

    # Resample polyline for better distance
    reference_path: np.ndarray = pops.sample_polyline(reference_path, 2)

    return reference_path
"""

if __name__ == "__main__":
    pass