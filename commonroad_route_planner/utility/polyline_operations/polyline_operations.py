import numpy as np


def compute_interpoint_distances_from_polyline(polyline: np.ndarray) -> np.ndarray:
    """
    Computes distance between to consecutive points
    :return: path length along polyline
    """
    assert (
        isinstance(polyline, np.ndarray)
        and polyline.ndim == 2
        and len(polyline[:, 0]) > 2
    ), "Polyline malformed for path lenth computation p={}".format(polyline)

    distance = np.zeros((len(polyline),))
    for i in range(1, len(polyline)):
        distance[i] = np.linalg.norm(polyline[i] - polyline[i - 1])

    return np.array(distance)


def compute_path_length_per_point(polyline: np.ndarray) -> np.ndarray:
    """
    Computes the path length of a polyline, i.e. the reference path
    :return: path length along polyline
    """
    assert (
        isinstance(polyline, np.ndarray)
        and polyline.ndim == 2
        and len(polyline[:, 0]) > 2
    ), "Polyline malformed for path lenth computation p={}".format(polyline)

    distance = np.zeros((len(polyline),))
    for i in range(1, len(polyline)):
        distance[i] = distance[i - 1] + np.linalg.norm(polyline[i] - polyline[i - 1])

    return np.array(distance)


def compute_scalar_curvature_from_polyline(polyline: np.ndarray) -> np.ndarray:
    """
    Computes scalar curvature along a polyline.

    :param polyline: polyline for which curvature should be calculated
    :return: curvature along  polyline
    """
    assert (
        isinstance(polyline, np.ndarray)
        and polyline.ndim == 2
        and len(polyline[:, 0]) > 2
    ), "Polyline malformed for curvature computation p={}".format(polyline)

    # Derivation to position, not time
    x_d: np.ndarray = np.gradient(polyline[:, 0])
    x_dd: np.ndarray = np.gradient(x_d)
    y_d: np.ndarray = np.gradient(polyline[:, 1])
    y_dd: np.ndarray = np.gradient(y_d)

    curvature_array: np.ndarray = (x_d * y_dd - x_dd * y_d) / (
        (x_d**2 + y_d**2) ** (3.0 / 2.0)
    )

    return curvature_array


def compute_orientation_from_polyline(polyline: np.ndarray) -> np.ndarray:
    """
    Computes orientation along a polyline

    :param polyline: polyline for which orientation should be calculated
    :return: orientation along polyline
    """
    assert (
        isinstance(polyline, np.ndarray)
        and len(polyline) > 1
        and polyline.ndim == 2
        and len(polyline[0, :]) == 2
    ), "<Math>: not a valid polyline. polyline = {}".format(polyline)
    if len(polyline) < 2:
        raise ValueError("Cannot create orientation from polyline of length < 2")

    orientation = [0]
    for i in range(1, len(polyline)):
        pt1 = polyline[i - 1]
        pt2 = polyline[i]
        tmp = pt2 - pt1
        orientation.append(np.arctan2(tmp[1], tmp[0]))

    return np.array(orientation)


def remove_duplicate_points(reference_path: np.ndarray) -> np.ndarray:
    """
    Removes identical points from reference path to avoid issues with numerical differenciation and curvature
    """
    _, idx = np.unique(reference_path, axis=0, return_index=True)
    ref_path = reference_path[np.sort(idx)]

    return ref_path


def compute_length_of_polyline(polyline: np.ndarray) -> float:
    """
    Computes length of reference path
    """
    inter_point_distance: np.ndarray = compute_interpoint_distances_from_polyline(
        polyline
    )
    return float(np.sum(inter_point_distance))


def sample_polyline(polyline: np.ndarray, step: float = 2.0) -> np.ndarray:
    """
    Samples polyline with a given step in meter.
    """
    return resample_polyline(polyline, step)


def resample_polyline(polyline: np.ndarray, step: float = 2.0) -> np.ndarray:
    """Resamples the input polyline with the specified step size.

    The distances between each pair of consecutive vertices are examined. If it is larger than the step size,
    a new sample is added in between.

    :param polyline: polyline with 2D points
    :param step: minimum distance between each consecutive pairs of vertices
    :return: resampled polyline
    """
    if len(polyline) < 2:
        return np.array(polyline)

    polyline_new = [polyline[0]]

    current_idx = 0
    current_position = step
    current_distance = np.linalg.norm(polyline[0] - polyline[1])

    # iterate through all pairs of vertices of the polyline
    while current_idx < len(polyline) - 1:
        if current_position <= current_distance:
            # add new sample and increase current position
            ratio = current_position / current_distance
            polyline_new.append(
                (1 - ratio) * polyline[current_idx] + ratio * polyline[current_idx + 1]
            )
            current_position += step

        else:
            # move on to the next pair of vertices
            current_idx += 1
            # if we are out of vertices, then break
            if current_idx >= len(polyline) - 1:
                break
            # deduct the distance of previous vertices from the position
            current_position = current_position - current_distance
            # compute new distances of vertices
            current_distance = np.linalg.norm(
                polyline[current_idx + 1] - polyline[current_idx]
            )

    # add the last vertex
    polyline_new.append(polyline[-1])

    return np.array(polyline_new)
