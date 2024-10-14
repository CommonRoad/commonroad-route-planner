The subsequent code snippet shows the general usage of the CommonRoad Route Planner.
Further examples can be found [here](https://github.com/commonroad/commonroad-route-planner/tutorials).

```Python
# commonrad
from commonroad.common.file_reader import CommonRoadFileReader

# Own Code base
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.utility.visualization import visualize_route
from commonroad_route_planner.frenet_tools.route_extendor import RouteExtendor
from commonroad_route_planner.lanelet_sequence import LaneletSequence
from commonroad_route_planner.reference_path_planner import ReferencePathPlanner
from commonroad_route_planner.reference_path import ReferencePath


# typing
from typing import List



def main(path_to_xml: str, save_imgs: bool = False, save_path: str = ""):
    # ========== Load the CommonRoad Scenario Object=========== #
    scenario, planning_problem_set = CommonRoadFileReader(
        f"{path_to_xml}"
    ).open()
    # retrieve the first planning problem in the problem set
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
    

    # ========== reference_path planning =========== #
    # Instantiate reference path and plan reference_path as lanelet sequence
    route_planner = RoutePlanner(
            lanelet_network=scenario.lanelet_network,
            planning_problem=planning_problem,
    )
    routes: List[LaneletSequence] = route_planner.plan_routes()

    # Instantiate reference path planner and plan reference path
    ref_path_planner: ReferencePathPlanner = ReferencePathPlanner(
            lanelet_network=scenario.lanelet_network,
            planning_problem=planning_problem,
            routes=routes,
    )
    
    reference_path: ReferencePath = ref_path_planner.plan_shortest_reference_path(
            retrieve_shortest=True, consider_least_lance_changes=True
    )
    
    # Extend route and reference path if necessary, for example for frenet frame
    route_extendor: RouteExtendor = RouteExtendor(reference_path)
    route_extendor.extend_reference_path_at_start_and_end()
    
    # ========== visualization =========== #
    visualize_route(
        reference_path=reference_path,
        scenario=scenario,
        planning_problem=planning_problem,
        save_img=save_imgs,
        draw_route_lanelets=True,
        draw_reference_path=True,
    )


if __name__ == "__main__":
    path_to_xml: str = "absolute/path/to/cr/xml.xml"
    save_path: str = "absolute/path/to/saving/dir"
    main(save_imgs=True, path_to_xml=path_to_xml)
```
