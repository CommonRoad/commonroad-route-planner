import numpy as np
import warnings


# third party
from scipy.spatial.kdtree import KDTree


# commonroad
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.state import InitialState




# own code base
from commonroad_route_planner.route import Route
from commonroad_route_planner.utility.route_util import (sort_lanelet_ids_by_goal,
                                                    sort_lanelet_ids_by_orientation)



# typing
from typing import List, Set, Tuple, Union



class RouteCandidateHolder:
    """
    [WARNING: Will be deprecated in future releases]
    
    Holds the route candidate and acts as a preprocessor
    """

    def __init__(self, lanelet_network: LaneletNetwork,
                 state_initial: InitialState, goal_region: GoalRegion,
                 route_candidates: List[List[int]], ids_lanelets_permissible: Set):
        
        warnings.warn(f'[DEPRECATION WARNING] will be deprecated in future release. Use RouteSelector (same method calls) instead')

        self.lanelet_network: LaneletNetwork = lanelet_network
        self.state_initial: InitialState = state_initial
        self.goal_region: GoalRegion = goal_region

        # create a list of Route objects for all routes found by the route planner which is not empty
        self.route_candidates: List = [
            Route(lanelet_network, route, ids_lanelets_permissible)
            for route in route_candidates if route
        ]
        self.num_route_candidates: int = len(self.route_candidates)

        # set permissible lanelet ids
        if ids_lanelets_permissible is None:
            self.ids_lanelets_permissible: Set = {
                lanelet.lanelet_id for lanelet in self.lanelet_network.lanelets
            }
        else:
            self.ids_lanelets_permissible: Set = ids_lanelets_permissible


    def retrieve_first_route(self,
                             retrieve_shortest: bool = True) -> Route:
        """
        Retrieves the first Route object.

        If retrieve shortest, the shortest route is used and orientation of the lanelet is checked.
        """        
        if(len(self.route_candidates) == 0):
            raise ValueError(f'[CR Route Planner] Not a single route candidate was found')

        elif(len(self.route_candidates) == 1 or not retrieve_shortest):
            return self.route_candidates[0]

        else:
            sorted_routes: List[Route] = sorted(self.route_candidates, key=lambda x: x.length_reference_path, reverse=False)

            for route in sorted_routes:
                if(self._heuristic_check_matching_orientation_of_initial_state(route.reference_path)):
                    return route

            raise ValueError(f'[CR Route Planner] could not find a well oriented route. Perhaps increase distance threshold')


    def _heuristic_check_matching_orientation_of_initial_state(self, reference_path: np.ndarray,
                                                            distance_threshold_in_meters: float = 1.0) -> bool:
        """
        Necessary to filter out the corner case, where the initial position is on multiple lanelets (i.e. on an
        intersection) and the shortest path might choose the wrong one
        """

        # Algorithm
        # ----------
        # use KDTree to check for closest point on ref path
        distance, idx = KDTree(reference_path).query(self.state_initial.position)

        if(distance <= distance_threshold_in_meters):
            return True
        else:
            return False


    def retrieve_all_routes(self) -> Tuple[List[Route], int]:
        """Returns the list of Route objects and the total number of routes"""
        return self.route_candidates, self.num_route_candidates
    
    

    def __repr__(self):
        return (
            f"RouteCandidateHolder(#candidates:{len(self.route_candidates)},initial_state={self.state_initial},"
            f"goal_region={self.goal_region})"
        )


    def __str__(self):
        return self.__repr__()













