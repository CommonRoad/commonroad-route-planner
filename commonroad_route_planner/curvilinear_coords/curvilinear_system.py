
import numpy as np




class CurvilinearCoordinateSystem():
    """
    Class that generates a curvilinear coordinate frame along a polyline.
    Useful here so that there is no dependency of the drivability checker.
    """

    def __init__(self, reference_path: np.ndarray):
        self.reference_path: np.ndarray = reference_path




