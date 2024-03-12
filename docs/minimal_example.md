The subsequent code snippet shows a minimal use case example
Further examples can be found [here](https://github.com/commonroad/commonroad-route-planner/tutorials).

```Python
# commonrad
from commonroad.common.file_reader import CommonRoadFileReader

# Own Code base
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.route import Route
from commonroad_route_planner.utility.visualization import visualize_route
from commonroad_route_planner.frenet_tools.route_extendor import RouteExtendor

# typing
from typing import TYPE_CHECKING




def main(path_to_xml: str, save_imgs: bool = False, save_path: str = ""):
    # ========== Load the CommonRoad Scenario Object=========== #
    scenario, planning_problem_set = CommonRoadFileReader(
        f"{path_to_xml}"
    ).open()
    # retrieve the first planning problem in the problem set
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]


    # Get route object
    route: Route = RoutePlanner(scenario, planning_problem).plan_routes().retrieve_shortest_route()


if __name__ == "__main__":
    path_to_xml: str = "absolute/path/to/cr/xml.xml"
    save_path: str = "absolute/path/to/saving/dir"
    main(save_imgs=True, path_to_xml=path_to_xml)
```
