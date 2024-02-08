from typing import Any, List, Dict, Set, Optional, Tuple

import swacon.configuration.environment as environment


# TODO: too complex, simplify, break into smaller parts
def convert_coordinates(coords: Tuple[float, float], x_high_new, x_low_new, y_high_new, y_low_new, x_high_old, x_low_old, y_high_old, y_low_old) -> Tuple[float, float]:
    # coords.x =  (xOld-xLoOld) / (xHiOld-xLoOld) * (xHiNew-xLoNew) + xLoNew
    xs, ys = coords
    xs = (xs - x_low_old) / (x_high_old - x_low_old) * (x_high_new - x_low_new) + x_low_new
    ys = (ys - y_low_old) / (y_high_old - y_low_old) * (y_high_new - y_low_new) + y_low_new
    return (xs, ys)


def simulation_to_raster(s: Tuple[float, float]) -> Tuple[float, float]:
    xs, ys = s
    assert isinstance(xs, float)
    assert isinstance(ys, float)
    xr = xs / environment.PIXELS_PER_TILE
    yr = ys / environment.PIXELS_PER_TILE
    return xr, yr


def raster_to_simulation(r: Tuple[float, float]) -> Tuple[float, float]:
    xr, yr = r
    assert isinstance(xr, float)
    assert isinstance(yr, float)
    xs = environment.PIXELS_PER_TILE * xr
    ys = environment.PIXELS_PER_TILE * yr
    return xs, ys


def raster_to_physical(r: Tuple[float, float]) -> Tuple[float, float]:
    xr, yr = r
    assert isinstance(xr, float)
    assert isinstance(yr, float)
    return convert_coordinates(r, 1, -1, -1, 1, 100, 0, 100, 0)
    xp = xr / TILES_PER_METER
    yp = yr / TILES_PER_METER
    xp += 1.0
    yp += 1.0
    return xp, yp


def physical_to_raster(p: Tuple[float, float]) -> Tuple[float, float]:
    xp, yp = p
    assert isinstance(xp, float)
    assert isinstance(yp, float)
    return convert_coordinates(p, 100, 0, 100, 0, 1, -1, -1, 1)
    xp += 1.0
    yp += 1.0
    xr = TILES_PER_METER * xp
    yr = TILES_PER_METER * yp
    return xr, yr
