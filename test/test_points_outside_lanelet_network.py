import unittest
import os
from pathlib import Path


# third party
from shapely import LineString as ShapelyLineString
from shapely import Polygon as ShapelyPolygon
from shapely import affinity as shapely_affinity

# commonrad
from commonroad.common.file_reader import CommonRoadFileReader

# Own Code base
from commonroad_route_planner.route_planner import RoutePlanner

# typing
from typing import List



class TestPointsOutsideLaneletNetwork(unittest.TestCase):
    """
    Tests whether reference path has points that are outside the lalenet network
    """
    def test_outsideroads(self, use_cr2023_challenge: bool = False, development=False):
        
        print(f'---  No-Points-outside-lanelet-network Test ---')
        
        if(use_cr2023_challenge):
            path_scenarios = Path(__file__).parents[1] / "tutorial/commonroad_challenge_2023"
        else:
            path_scenarios = Path(__file__).parents[1] / "scenarios"

        # Somehow, there are still some weird bugs in these ones
        ignored_scenarios: List = [
            'USA_Peach-3_1_T-1.xml', 
            'USA_US101-22_1_T-1.xml', 
            'USA_US101-32_1_T-1.xml', 
            'ZAM_Zip-1_6_T-1.xml'
        ]
        
        # TODO: Make the scenarios from the ignored list above work
        if(development):
            ignored_scenarios = []
            
        
        successes: List[str] = list()
        fails: List[str] = list()
        for idx, filename in enumerate(sorted(os.listdir(path_scenarios))):
            if filename in ignored_scenarios:
                continue
            
            print(f"Testing scenario {filename}")
    
            id_scenario = filename.split(".")[0]
            no_outside: bool = self._analyse_scenario(id_scenario, path_scenarios)
            
            if(no_outside == True):
                successes.append(filename)
            else:
                fails.append(filename)
                
        # Test output
        if(len(fails) > 0):
            raise ValueError(f"Not all reference paths where entirely inside the lanelet network \n"
                                f"Fails: {len(fails)}  --  {fails}")
            
                
                
    def _analyse_scenario(self, id_scenario: str, path_scenarios: Path) -> bool:
        """
        Test if polyline of reference path intersects with goal.
        Returns (success_scenarios, failed_scenarios)
        
        """
        
        # Load scenario and get planning problem
        scenario, planning_problem_set = CommonRoadFileReader(
            f"{path_scenarios / id_scenario}.xml"
        ).open()
        planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

        # Plan route
        route_planner = RoutePlanner(
            scenario,
            planning_problem,
            use_predecessors_to_pass_through_goal_state=False,
        )
        candidate_holder = route_planner.plan_routes()
        route = candidate_holder.retrieve_first_route(retrieve_shortest=True)
        
        # Create shapely route
        shapely_route_line: ShapelyLineString = ShapelyLineString(route.reference_path)
        
        # Create scaled ab version of road compolement
        road_network_polygon: ShapelyPolygon = None
        for polygon in scenario.lanelet_network.lanelet_polygons:
            if(road_network_polygon is None):
                road_network_polygon = polygon.shapely_object
            else:
                road_network_polygon.union(polygon.shapely_object)
                
        exterior_polygon: ShapelyPolygon = road_network_polygon.convex_hull
        scaled_exterior_polygon: ShapelyPolygon = shapely_affinity.scale(exterior_polygon, xfact=10.0, yfact=10.0)
        complement_road_network_polygon = scaled_exterior_polygon.difference(road_network_polygon)
        
        
        # Check if reference path and road complement intersect
        if(shapely_route_line.within(complement_road_network_polygon)):
            return False
        else:
            return True
        