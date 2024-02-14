from copy import deepcopy
from typing import Dict, List, Optional, Tuple

import xarray
import numpy as np
import shapely as sh
from numpy.ma import make_mask
from shapely.ops import voronoi_diagram
from rasterio.features import rasterize

from swacon.algorithms.transforms.coordinate_transforms import physical_to_raster, raster_to_physical
from swacon.data_structures.drone.state import DroneState


# Floating point value under this is considered to be zero
EPSILON = 0.0001


def solve_central_mass_point(  # noqa: C901
    name: str, drone_states: Dict[str, DroneState], environment: xarray.DataArray, envelope: sh.Polygon, tiles_in_physical_x: int, tiles_in_physical_y: int
) -> Optional[Tuple[float, float]]:
    """Solve central mass point for a drone in given environment

    ### Arguments
    - `name`: the name of the drone
    - `drone_states`: the states of all drones
    - `environment`: the importance field where the drone operates
    - `envelope`: must be the laboratory raster rectangle
    - `tiles_in_physical_x`: the amount of tiles in physical x-axis
    - `tiles_in_physical_y`: the amount of tiles in physical y-axis

    ### Result
    - None if no solution was found
    - (x, y) if central mass point was found
    """
    assert isinstance(name, str)
    assert isinstance(drone_states, dict)
    assert isinstance(environment, xarray.DataArray)
    assert isinstance(envelope, sh.Polygon)
    assert isinstance(tiles_in_physical_x, int)
    assert isinstance(tiles_in_physical_y, int)
    # Solve the voronoi polygons for all drones
    points: List[sh.Point] = list()
    for i_name, i_state in drone_states.items():
        # Physical position
        p = (i_state.x, i_state.y)
        # Raster position
        r = physical_to_raster(p)
        xr, yr = r
        point = sh.Point(xr, yr)
        if point.is_empty:
            print(f"[{name}] drone {i_name} point is empty")
            return None
        points.append(point)
    multipoint = sh.MultiPoint(points)
    diagram = voronoi_diagram(multipoint, envelope=envelope)
    assert isinstance(diagram, sh.GeometryCollection)
    if len(list(diagram.geoms)) <= 0:
        print(f"[{name}] voronoi diagram geometry collection is empty")
        print(f"  - {points}")
        return None
    # Physical position
    drone_state = drone_states.get(name)
    if drone_state is None:
        return None
    p = (drone_state.x, drone_state.y)
    xp, yp = p
    # Raster position
    r = physical_to_raster(p)
    xr, yr = r
    pi = sh.Point(xr, yr)
    polygon_i = None
    for geometry in diagram.geoms:
        # Clip to environment bounds
        polygon = envelope.intersection(geometry)
        assert isinstance(polygon, sh.Polygon)
        if polygon.contains(pi):
            polygon_i = polygon
            break
    if polygon_i is None:
        print(f"[{name}] polygon i is none")
        return None
    raster = rasterize([polygon_i], (tiles_in_physical_x, tiles_in_physical_y)).astype(bool)
    if np.all(raster == 0.0):
        # TODO: how to best handle this error?
        print(f"[{name}] non-zero raster not found")
        raise Exception()
    mask = make_mask(raster)
    A = deepcopy(environment.data)
    A[~mask] = 0.0
    xs, ys = np.nonzero(0 < A)
    # Solve center of mass of drone i's Voronoi polygon
    total_mass = 0.0
    upper = np.zeros((2, 1))
    for xpi, ypi in zip(xs, ys):
        r = np.array([ypi, xpi]).reshape((2, 1))
        point_mass = A[xpi, ypi]
        total_mass += point_mass
        upper += point_mass * r
    if total_mass < EPSILON:
        print(f"[{name}] voronoi region total mass was zero")
        return None
    else:
        # Raster position
        center_of_mass = upper / total_mass
        rx, ry = center_of_mass
        rx, ry = float(rx), float(ry)
        rp = (rx, ry)
        # Physical position
        pp = raster_to_physical(rp)
        # Record Voronoi polygon
        # TODO: what if drone is removed?
        # with VORONOI_POLYGONS_LOCK:
        #    VORONOI_POLYGONS[drone.name] = polygon_i
        # with VORONOI_POLYGONS_CENTRAL_MASSES_LOCK:
        #    VORONOI_POLYGONS_CENTRAL_MASSES[drone.name] = (float(x), float(y))
        return pp


def solve_control(
    name: str, drone_states: Dict[str, DroneState], environment: xarray.DataArray, envelope: sh.Polygon, tiles_in_physical_x: int, tiles_in_physical_y: int, k_proportional: float
) -> Optional[Tuple[float, float, float, float]]:
    """Solve central mass point for a drone in given environment

    ### Arguments
    - `name`: the name of the drone
    - `drone_states`: the states of all drones
    - `environment`: the importance field where the drone operates
    - `envelope`: must be the laboratory raster rectangle
    - `tiles_in_physical_x`: the amount of tiles in physical x-axis
    - `tiles_in_physical_y`: the amount of tiles in physical y-axis
    - `k_proportional`: the proportional control multiplier

    ### Result
    - None if no solution was found
    - (x, y, vx, vy) if central mass point was found
    """
    assert isinstance(drone_states, dict)
    drone_state = drone_states.get(name)
    if drone_state is None:
        return None
    cmp = solve_central_mass_point(name, drone_states, environment, envelope, tiles_in_physical_x, tiles_in_physical_y)
    if cmp is None:
        return None
    # Physical position
    cx, cy = cmp
    # Physical position
    px = drone_state.x
    py = drone_state.y
    # Physical position
    vx = -1.0 * k_proportional * (px - cx)
    vy = -1.0 * k_proportional * (py - cy)
    return (cx, cy, vx, vy)
