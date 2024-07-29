class NoSourceLaneletIdException(Exception):
    def __init__(self):
        self.message = "<RoutePlanner> No initial position given."


class NoPathFoundException(Exception):
    def __init__(self, message):
        self.message = message


class PointNotOnGoalLaneletException(Exception):
    def __init__(self, planning_problem_id: int, goal_lanelet_id: int):
        self.message = (
            f"<Route Planner> for planning problem id {planning_problem_id} the reference path does not end "
            f"at the goal lanelet id {goal_lanelet_id}"
        )
