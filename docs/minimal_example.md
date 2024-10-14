The subsequent code snippet shows a minimal use case example
Further examples can be found [here](https://github.com/commonroad/commonroad-route-planner/tutorials).

```Python
# commonrad
from commonroad.common.file_reader import CommonRoadFileReader

# Own Code base
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.reference_path import ReferencePath
from commonroad_route_planner.reference_path_planner import ReferencePathPlanner
from commonroad_route_planner.lanelet_sequence import LaneletSequence

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
    
    
if __name__ == "__main__":
    path_to_xml: str = "absolute/path/to/cr/xml.xml"
    save_path: str = "absolute/path/to/saving/dir"
    main(save_imgs=True, path_to_xml=path_to_xml)
```
