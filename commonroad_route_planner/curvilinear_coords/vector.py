import math

import numpy as np


class Vector2D:
    """
    2D Vector
    """

    @classmethod
    def calculate_orientation_between_vectors(cls,
                                              vector_1: "Vector2D",
                                              vector_2: "Vector2D"
                                              ) -> float:
        """
        Calcs orientation between to vectors

        :param vector_1: first vector
        :param vector_2: second vector

        :return: orientation in rad
        """
        theta_rad: float = math.acos(
            np.dot(vector_1._entries, vector_2._entries) / (vector_1._norm * vector_2._norm)
        )

        return theta_rad

    def __init__(self,
                 p_start: np.ndarray,
                 p_end: np.ndarray
                 ) -> None:
        """
        2D Vector

        :param p_start: (2,) np array of start point
        :param p_end: (2,) np array of end point
        """

        self._p_start: np.ndarray = p_start
        self._p_end: np.ndarray = p_end

        self._entries: np.ndarray = np.asarray([p_end[0] - p_start[0], p_end[1] - p_start[1]])
        self._norm: float = math.sqrt((self._entries[0]) ** 2 + (self._entries[1]) ** 2)

        # FIXME: Only ever gets smaller angle, we need the correct one!
        # Check: https://stackoverflow.com/questions/14066933/direct-way-of-computing-the-clockwise-angle-between-two-vectors
        dot = self._entries[0]
        det = self._entries[1]
        self._angle_to_x = math.atan2(det, dot)

        if(self._norm == 0):
            raise ValueError(f'Norm of vector is zero. This class is not meant for zero vectors.')


    @property
    def p_start(self) -> np.ndarray:
        """
        :return: (2,) array of vector start point
        """
        return self._p_start

    @property
    def p_end(self) -> np.ndarray:
        """
        :return: (2,) array of vector end point
        """
        return self._p_end

    @property
    def entries(self) -> np.ndarray:
        """
        :return: (2,) array of vector entries
        """
        return self._entries


    @property
    def norm(self) -> float:
        """
        :return: (2,) array of vector entries
        """
        return self._norm


    @property
    def angle_to_x(self) -> float:
        """
        :return: angle between vector and x axis
        """
        return self._angle_to_x







