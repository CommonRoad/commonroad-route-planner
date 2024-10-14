The subsequent code snippet shows specific use cases for Planning in Frenet Frame.

```Python
# commonrad
from commonroad.common.file_reader import CommonRoadFileReader

# Own Code base
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.lanelet_sequence import LaneletSequence
from commonroad_route_planner.reference_path_planner import ReferencePathPlanner
from commonroad_route_planner.reference_path import ReferencePath
from commonroad_route_planner.frenet_tools.route_extendor import RouteExtendor

# typing
from typing import List



def main(path_to_xml: str):
    # ========== Load the CommonRoad Scenario Object=========== #
    scenario, planning_problem_set = CommonRoadFileReader(
        f"{path_to_xml}"
    ).open()
    # retrieve the first planning problem in the problem set
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    # Route
    route_planner = RoutePlanner(
        lanelet_network=scenario.lanelet_network,
        planning_problem=planning_problem,
    )
    routes: List[LaneletSequence] = route_planner.plan_routes()

    # Reference path
    ref_path_planner: ReferencePathPlanner = ReferencePathPlanner(
        lanelet_network=scenario.lanelet_network,
        planning_problem=planning_problem,
        routes=routes,
    )
    reference_path: ReferencePath = ref_path_planner.plan_shortest_reference_path(
        retrieve_shortest=True, consider_least_lance_changes=True
    )

    # ========== For Planning in Frenet Frame, it might be useful to extend the reference_path a bit =========== #
    # Init reference_path extendor
    route_extendor: RouteExtendor = RouteExtendor(reference_path)
    # Extend reference path at start and end
    route_extendor.extend_reference_path_at_start_and_end()



if __name__ == "__main__":
    path_to_xml: str = "absolute/path/to/cr/xml.xml"
    save_path: str = "absolute/path/to/saving/dir"
    main(save_imgs=True, path_to_xml=path_to_xml)
```
