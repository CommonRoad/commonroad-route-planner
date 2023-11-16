import logging
from typing import List

import gurobipy as gp
import numpy as np
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.state import State


class MapMatcher:
    def __init__(
        self, lanelet_network: LaneletNetwork, relax_consistency_constraint: int = 2
    ):
        self.lanelet_network: LaneletNetwork = lanelet_network
        self.relax_consistency_constraint: int = relax_consistency_constraint

    def map_mathing(self, trajectory: List[State]) -> List[int]:
        """
        Conduct map matching for given trajectory.
        """
        # determine occupancy matrix
        occupancy_dict = dict()
        occurring_lanelet_ids = set()
        for k in range(len(trajectory)):
            occupancy_dict[k] = self.lanelet_network.find_lanelet_by_position(
                [trajectory[k].position]
            )[0]
            for lt_id in occupancy_dict[k]:
                occurring_lanelet_ids.add(lt_id)

        occurring_lanelet_ids = sorted(occurring_lanelet_ids)
        lanelet_mapping = dict(
            zip(occurring_lanelet_ids, range(len(occurring_lanelet_ids)))
        )
        occupancy_matrix = np.zeros((len(occurring_lanelet_ids), len(trajectory)), bool)
        for key, val in occupancy_dict.items():
            for v in val:
                occupancy_matrix[lanelet_mapping[v], key] = True

        # optimization problem
        m = gp.Model(
            "Map Matching"
        )  # academic gurobi lic https://support.gurobi.com/hc/en-us/articles/360040541251

        number_time_steps = len(trajectory)
        number_lanelets = len(occurring_lanelet_ids)

        # decision variable
        x = m.addMVar(np.shape(occupancy_matrix), vtype=gp.GRB.BINARY, name="x")

        def objective_function(x: np.ndarray) -> int:
            # choose the lanelets where the vehicle can drive the longest
            return -gp.quicksum(gp.quicksum(x.T) * gp.quicksum(x.T))

        # objective
        m.setObjective(objective_function(x), gp.GRB.MINIMIZE)

        # constraints
        m.addConstr(
            x[:, :] <= occupancy_matrix[:, :]
        )  # only choose lanelets that are available

        for k in range(number_time_steps):
            m.addConstr(
                gp.quicksum(x[:, k]) <= 1
            )  # only match one lanelet at each timestep

        # consider lanelet network topology
        for lt_id in occurring_lanelet_ids:
            preceding_lanelets = self.lanelet_network.find_lanelet_by_id(
                lt_id
            ).predecessor.copy()

            to_be_removed = set()
            for j in range(len(preceding_lanelets)):
                if preceding_lanelets[j] in lanelet_mapping.keys():
                    preceding_lanelets[j] = lanelet_mapping[preceding_lanelets[j]]
                else:
                    to_be_removed.add(preceding_lanelets[j])  # entry can be skipped

            for v in to_be_removed:  # start with deleting at the end
                preceding_lanelets.remove(v)

            preceding_lanelets.append(
                lanelet_mapping[lt_id]
            )  # one can also continue on the same lanelet

            for k in range(number_time_steps - 1 - self.relax_consistency_constraint):
                # occupancy of all preceding lanelets (and oneself) >= current lanelet
                helper = []
                for pl in preceding_lanelets:
                    helper.append(pl)

                m.addConstr(
                    gp.quicksum(
                        gp.quicksum(
                            x[
                                preceding_lanelets,
                                k : k + 1 + self.relax_consistency_constraint,
                            ]
                        )
                    )
                    >= x[
                        lanelet_mapping[lt_id],
                        k + 1 + self.relax_consistency_constraint,
                    ]
                )

        m.optimize()

        if m.Status != 2:
            #  Solution status not optimal.
            #  Maybe try higher value for relax_consistency
            raise NotImplementedError

        raw_result = np.reshape(m.getAttr("x"), (number_lanelets, number_time_steps))
        # lanelet_trajectory = np.argmax(raw_result, axis=0)
        lanelet_trajectory = []
        for k in range(number_time_steps):
            ind = np.argmax(raw_result[:, k])
            if raw_result[ind, k] > 0:
                lanelet_trajectory.append(occurring_lanelet_ids[ind])
            else:
                logging.warning("No map matching at time step: {}".format(k))

        lanelet_sequence = [lanelet_trajectory[0]]
        for k in range(len(lanelet_trajectory) - 1):
            if lanelet_trajectory[k + 1] != lanelet_sequence[-1]:
                lanelet_sequence.append(lanelet_trajectory[k + 1])

        return lanelet_sequence
