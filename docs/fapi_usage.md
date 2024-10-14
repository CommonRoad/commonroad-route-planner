The subsequent code snippet shows an exemplary fast api usage
Further examples can be found [here](https://github.com/commonroad/commonroad-route-planner/tutorials).

```Python
# commonrad
from commonroad.common.file_reader import CommonRoadFileReader

# Own Code base
import commonroad_route_planner.fast_api.fast_api as fast_api

def main(path_to_xml: str):
    # ========== Load the CommonRoad Scenario Object=========== #
    scenario, planning_problem_set = CommonRoadFileReader(
    f"{path_to_xml}"
    ).open()
    # retrieve the first planning problem in the problem set
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
    
    fast_api.generate_reference_path_from_lanelet_network_and_planning_problem(
        lanelet_network=scenario.lanelet_network,
        planning_problem=planning_problem
    )
    
    
if __name__ == "__main__":
    path_to_xml: str = "absolute/path/to/cr/xml.xml"
    save_path: str = "absolute/path/to/saving/dir"
    main(save_imgs=True, path_to_xml=path_to_xml)
```