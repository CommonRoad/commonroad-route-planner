The subsequent code snippet shows the general usage of the CommonRoad Route Planner.
Further examples can be found [here](https://github.com/commonroad/commonroad-route-planner/tutorials).

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
    from commonroad_route_planner.route_selector import RouteSelector


def main(path_to_xml: str, save_imgs: bool = False, save_path: str = ""):
    # ========== Load the CommonRoad Scenario Object=========== #
    scenario, planning_problem_set = CommonRoadFileReader(
        f"{path_to_xml}"
    ).open()
    # retrieve the first planning problem in the problem set
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    # ========== route planning =========== #
    # instantiate a route planner with the scenario and the planning problem
    route_planner = RoutePlanner(
        scenario=scenario,
        planning_problem=planning_problem,
        extended_search=False
    )
    # plan routes, and save the routes in a route candidate holder
    route_selector: "RouteSelector" = route_planner.plan_routes()

    # ========== retrieving the best route =========== #
    # here we retrieve the shortest route that has the least amount of disjoint lane changes
    route: "Route" = route_selector.retrieve_shortest_route(
        retrieve_shortest=True,
        consider_least_lance_changes=True
    )

    # ========== For Planning in Frenet Frame, it might be useful to extend the route a bit =========== #
    # Init route extendor
    route_extendor: RouteExtendor = RouteExtendor(route)
    # Extend reference path at start and end
    route_extendor.extend_reference_path_at_start_and_end()

    # ========== You can also retrieve all routes =========== #
    list_routes, num_routes_retrieved = route_selector.retrieve_all_routes()
    print(f"Number of routes retrieved: {num_routes_retrieved}")

    # ========== visualization =========== #
    visualize_route(
        route=route,
        scenario=scenario,
        planning_problem=planning_problem,
        save_img=save_imgs,
        draw_route_lanelets=True,
        draw_reference_path=True,
        save_path=save_path,
    )


if __name__ == "__main__":
    path_to_xml: str = "absolute/path/to/cr/xml.xml"
    save_path: str = "absolute/path/to/saving/dir"
    main(save_imgs=True, path_to_xml=path_to_xml)
```