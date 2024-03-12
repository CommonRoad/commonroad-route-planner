__author__ = "Florian Finkeldei"

import logging
from typing import List

import cvxpy as cp
import numpy as np
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.state import InitialState, State


class MapMatcher:
    """
    Takes a given trajectory from the past and tries to match the points to lanelets
    """

    def __init__(self, lanelet_network: LaneletNetwork):
        self.lanelet_network: LaneletNetwork = lanelet_network

    def map_matching(
        self,
        trajectory: List[State],
        initial_state: InitialState = None,
        allow_diagonal_transition: bool = False,
        relax_consistency_constraint: int = 0,
    ) -> List[int]:
        """
        Conduct map matching for given trajectory.
        """
        # determine occupancy matrix
        occupancy_dict = dict()
        occurring_lanelet_ids = set()
        if initial_state is not None:
            # add initial state to trajectory
            assert initial_state.time_step == trajectory[0].time_step - 1
            trajectory.insert(0, initial_state)

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

        number_time_steps = len(trajectory)

        # decision variable
        x = cp.Variable(np.shape(occupancy_matrix), boolean=True)

        # constraints
        constr = [
            x[:, :] <= occupancy_matrix[:, :]
        ]  # only choose lanelets that are available

        for k in range(number_time_steps):
            constr += [
                cp.sum(x[:, k]) <= 1
            ]  # at most match one lanelet at each timestep

        for k in range(number_time_steps):
            upper_lim = min(k + relax_consistency_constraint + 1, number_time_steps)
            constr += [cp.sum(x[:, k:upper_lim]) >= 1]  # relax

        # consider lanelet network topology
        for lt_id in occurring_lanelet_ids:
            lanelet = self.lanelet_network.find_lanelet_by_id(lt_id)
            preceding_lanelets = lanelet.predecessor.copy()

            if allow_diagonal_transition:
                # also add adjacent lanelets with same direction from predecessors -- "diagonal transition"
                for pred_it in lanelet.predecessor:
                    pred = self.lanelet_network.find_lanelet_by_id(pred_it)
                    if pred.adj_left is not None:
                        if pred.adj_left_same_direction:
                            preceding_lanelets.append(pred.adj_left)

                    if pred.adj_right is not None:
                        if pred.adj_right_same_direction:
                            preceding_lanelets.append(pred.adj_right)

            # also add adjacent lanelets with same direction:
            if lanelet.adj_left is not None:
                if lanelet.adj_left_same_direction:
                    preceding_lanelets.append(lanelet.adj_left)

            if lanelet.adj_right is not None:
                if lanelet.adj_right_same_direction:
                    preceding_lanelets.append(lanelet.adj_right)

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

            for k in range(number_time_steps - 1 - relax_consistency_constraint):
                # occupancy of all preceding lanelets (and oneself) >= current lanelet
                constr += [
                    cp.sum(
                        x[
                            preceding_lanelets,
                            k : k + 1 + relax_consistency_constraint,
                        ]
                    )
                    >= x[
                        lanelet_mapping[lt_id],
                        k + 1 + relax_consistency_constraint,
                    ]
                ]

        m = cp.Problem(
            cp.Minimize(cp.sum(cp.abs((cp.diff(x, axis=1)))) - cp.sum(x)),
            constraints=constr,
        )

        m.solve(cp.GLPK_MI)

        if m.status != "optimal":
            #  Solution status not optimal.
            #  Maybe try higher value for relax_consistency and allow diagonal transition.
            raise ValueError(
                "Try higher relax_consistency value or allow diagonal transitions."
            )

        raw_result = x.value
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

        if allow_diagonal_transition:
            lanelet_sequence = self.post_processing(lanelet_sequence)

        return lanelet_sequence

    def post_processing(self, lanelet_sequence: List[int]) -> List[int]:
        i = 0  # number of insertions
        for j in range(len(lanelet_sequence) - 1):
            if (
                lanelet_sequence[i + j + 1]
                in self.lanelet_network.find_lanelet_by_id(
                    lanelet_sequence[i + j]
                ).successor
            ):
                pass
            elif (
                self.lanelet_network.find_lanelet_by_id(
                    lanelet_sequence[i + j]
                ).adj_left
                is not None
            ):
                if self.lanelet_network.find_lanelet_by_id(
                    lanelet_sequence[i + j]
                ).adj_left_same_direction:
                    if (
                        lanelet_sequence[i + j + 1]
                        in self.lanelet_network.find_lanelet_by_id(
                            self.lanelet_network.find_lanelet_by_id(
                                lanelet_sequence[i + j]
                            ).adj_left
                        ).successor
                    ):
                        lanelet_sequence.insert(
                            i + j + 1,
                            self.lanelet_network.find_lanelet_by_id(
                                lanelet_sequence[i + j]
                            ).adj_left,
                        )
                        i += 1
                        logging.info("Correction applied.")
            elif (
                self.lanelet_network.find_lanelet_by_id(
                    lanelet_sequence[i + j]
                ).adj_right
                is not None
            ):
                if self.lanelet_network.find_lanelet_by_id(
                    lanelet_sequence[i + j]
                ).adj_right_same_direction:
                    if (
                        lanelet_sequence[i + j + 1]
                        in self.lanelet_network.find_lanelet_by_id(
                            self.lanelet_network.find_lanelet_by_id(
                                (lanelet_sequence[i + j])
                            ).adj_right
                        ).successor
                    ):
                        lanelet_sequence.insert(
                            i + j + 1,
                            self.lanelet_network.find_lanelet_by_id(
                                lanelet_sequence[i + j]
                            ).adj_right,
                        )
                        i += 1
                        logging.info("Correction applied.")
            else:
                raise ValueError  # could not repair / reconstruct lanelet sequence

        return lanelet_sequence
