The subsequent code snippet shows specific use cases for Planning in Frenet Frame.

```Python
# commonrad
from commonroad.common.file_reader import CommonRoadFileReader

# Own Code base
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.utility.visualization import visualize_route
from commonroad_route_planner.frenet_tools.route_extendor import RouteExtendor

# typing
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from commonroad_route_planner.route import Route
    from commonroad_route_planner.route_candidate_holder import RouteGenerator


def main(path_to_xml: str, save_imgs: bool = False, save_path: str = ""):
    # ========== Load the CommonRoad Scenario Object=========== #
    scenario, planning_problem_set = CommonRoadFileReader(
        f"{path_to_xml}"
    ).open()
    # retrieve the first planning problem in the problem set
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    # Get route object
    route: Route = RoutePlanner(scenario, planning_problem).plan_routes().retrieve_shortest_route()

    # ========== For Planning in Frenet Frame, it might be useful to extend the route a bit =========== #
    # Init route extendor
    route_extendor: RouteExtendor = RouteExtendor(route)
    # Extend reference path at start and end
    route_extendor.extend_reference_path_at_start_and_end()

    # This is unnecessary but shows that the route_extendor modified the route object
    route: Route = route_extendor.get_route()


if __name__ == "__main__":
    path_to_xml: str = "absolute/path/to/cr/xml.xml"
    save_path: str = "absolute/path/to/saving/dir"
    main(save_imgs=True, path_to_xml=path_to_xml)
```