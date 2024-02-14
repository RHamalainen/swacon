# TODO: background thread that attempts to connect to drones

# TODO: algoritmi ottaa huomioon myös dronet mitkä ei ole yhdistetty!!!!

# TODO: connect -> disconnect: meneekö synkronoinnissa jotain vikaan?

import sys
import time
from typing import Dict, List, Set, Optional, Tuple
from queue import Queue
from threading import Thread, Lock
from copy import deepcopy
from enum import Enum, unique, auto
import math

import numpy as np
import xarray as xr
from scipy.ndimage import gaussian_filter
import shapely as sh
from shapely.ops import voronoi_diagram
from rasterio.features import rasterize
from numpy.ma import make_mask
import matplotlib.pyplot as plt

import PySide6.QtCore as QtCore
from PySide6.QtCore import QRectF, Slot, QTimer, QPointF, QRect, QMargins, QMarginsF, Qt
from PySide6.QtGui import QFont, QColor, QCloseEvent, QMouseEvent, QKeyEvent, QPainterPath, QPainter, QWheelEvent
from PySide6.QtWidgets import QCompleter, QGridLayout, QLineEdit, QGraphicsScene, QGraphicsView, QWidget, QApplication, QGraphicsItem, QGraphicsRectItem, QLabel, QStyleOptionGraphicsItem


def create_environment(width: int, height: int):
    assert isinstance(width, int)
    assert isinstance(height, int)
    print(f"creating environment with width={width} and height={height}")

    Z = xr.DataArray(data=0.0, dims={"x": width, "y": height}, coords={"x": range(width), "y": range(height)})

    x = int(0.6 * width)
    y = int(0.6 * height)
    Z[x, y] = 1.0

    x = int(0.3 * width)
    y = int(0.5 * height)
    Z[x, y] = 1.0

    Z.data = gaussian_filter(Z, sigma=20)
    Z *= 1.0 / Z.max()
    # Extract target regions
    # threshold = 0.6
    # Z[Z < threshold] = 0
    # Assign uniform importance to each point
    # Z[0 < Z] = 1
    return Z


# Physical coordinates (meter)
LABORATORY_PHYSICAL_RECTANGLE = sh.Polygon(
    [
        (-1.0, -1.0),
        (1.0, -1.0),
        (1.0, 1.0),
        (-1.0, 1.0),
    ]
)
LABORATORY_PHYSICAL_RECTANGLE_WIDTH = 2.0
LABORATORY_PHYSICAL_RECTANGLE_HEIGHT = 2.0


def tf(x):
    x[:, 0] += 1.0
    x[:, 1] += 1.0

    print(x)
    return x


LABORATORY_RASTER_RECTANGLE = sh.transform(LABORATORY_PHYSICAL_RECTANGLE, tf)


quit()


# LABORATORY_PHYSICAL_TOP_LEFT = QPointF(-1.0, 1.0)
# LABORATORY_PHYSICAL_BOTTOM_RIGHT = QPointF(1.0, -1.0)
# LABORATORY_PHYSICAL_RECTANGLE = QRectF(
#    LABORATORY_PHYSICAL_TOP_LEFT,
#    LABORATORY_PHYSICAL_BOTTOM_RIGHT,
# )
# LABORATORY_PHYSICAL_RECTANGLE_WIDTH = abs(LABORATORY_PHYSICAL_RECTANGLE.width())
# LABORATORY_PHYSICAL_RECTANGLE_HEIGHT = abs(LABORATORY_PHYSICAL_RECTANGLE.height())

TILES_PER_METER = 50
METERS_PER_TILE = 1 / TILES_PER_METER
TILES_IN_PHYSICAL_X = math.ceil(TILES_PER_METER * LABORATORY_PHYSICAL_RECTANGLE_WIDTH)
TILES_IN_PHYSICAL_Y = math.ceil(TILES_PER_METER * LABORATORY_PHYSICAL_RECTANGLE_HEIGHT)
TILE_PHYSICAL_WIDTH = LABORATORY_PHYSICAL_RECTANGLE_WIDTH / TILES_IN_PHYSICAL_X
TILE_PHYSICAL_HEIGHT = LABORATORY_PHYSICAL_RECTANGLE_HEIGHT / TILES_IN_PHYSICAL_Y

PIXELS_PER_TILE = 10
TILES_PER_PIXEL = 1 / PIXELS_PER_TILE

# print(TILES_IN_PHYSICAL_X, TILES_IN_PHYSICAL_Y, TILE_PHYSICAL_WIDTH, TILE_PHYSICAL_HEIGHT)
# quit()

# Simulation coordinates
LABORATORY_SIMULATION_TOP_LEFT = QPointF(0.0, 0.0)
LABORATORY_SIMULATION_BOTTOM_RIGHT = QPointF(
    (TILES_IN_PHYSICAL_X + 0) * PIXELS_PER_TILE,
    (TILES_IN_PHYSICAL_Y + 0) * PIXELS_PER_TILE,
)
LABORATORY_SIMULATION_RECTANGLE = QRectF(LABORATORY_SIMULATION_TOP_LEFT, LABORATORY_SIMULATION_BOTTOM_RIGHT)


def physical_to_simulation(xp: float, yp: float) -> Tuple[float, float]:
    assert isinstance(xp, float)
    assert isinstance(yp, float)
    xs = xp + 1.0
    ys = yp - 1.0
    return xs, ys


def simulation_to_physical(xs: float, ys: float) -> Tuple[float, float]:
    assert isinstance(xs, float)
    assert isinstance(ys, float)
    xp = xs
    yp = ys
    return xp, yp


print(LABORATORY_PHYSICAL_RECTANGLE)
print(LABORATORY_SIMULATION_RECTANGLE)

print(physical_to_simulation(-1.0, 1.0))


quit()


# TODO: is valid?
ENVIRONMENT_POLYGON = sh.Polygon(
    [
        (-100.0, -100.0),
        (100.0, -100.0),
        (100.0, 100.0),
        (-100.0, 100.0),
    ]
)

VIEW_PADDING = 20
VIEW_MARGINS = QMarginsF(VIEW_PADDING, VIEW_PADDING, VIEW_PADDING, VIEW_PADDING)
VIEW_RECTANGLE = LABORATORY_SIMULATION_RECTANGLE.marginsAdded(VIEW_MARGINS)


NAME_TO_URI = {
    "cf1": "cf1_uri",
    "cf2": "cf2_uri",
    "cf3": "cf3_uri",
    "cf4": "cf4_uri",
    "cf5": "cf5_uri",
    "cf6": "cf6_uri",
}


class Command:
    def __init__(self):
        pass


class CommandQuit(Command):
    def __init__(self):
        super().__init__()


class CommandState(Command):
    def __init__(self):
        super().__init__()


class CommandStop(Command):
    def __init__(self):
        super().__init__()


class CommandAutopilot(Command):
    def __init__(self, state: bool):
        super().__init__()
        assert isinstance(state, bool)
        self.state = state


class CommandUpdateEnvironment(Command):
    def __init__(self, environment):
        super().__init__()
        self.environment = environment


# TODO: class Environment


@unique
class MoveType(Enum):
    ABSOLUTE = auto()
    RELATIVE = auto()


class CommandMove(Command):
    def __init__(self, move_type: MoveType, x: float, y: float, z: Optional[float] = None, angle: Optional[float] = None):
        super().__init__()
        assert isinstance(move_type, MoveType)
        assert isinstance(x, float)
        assert isinstance(y, float)
        self.move_type = move_type
        self.x = x
        self.y = y
        if z is not None:
            assert isinstance(z, float)
        self.z = z
        if angle is not None:
            assert isinstance(angle, float)
        self.angle = angle


class Drone:
    def __init__(self, name: str, uri: str, queue: Queue[Command]):
        assert isinstance(name, str)
        assert isinstance(uri, str)
        assert isinstance(queue, Queue)
        self.name = name
        self.uri = uri
        self.queue = queue
        self.maximum_speed = 10.0
        self.graphics_item: Optional[GraphicsDroneItem] = None


DRONE_ACTUAL_POSITIONS_LOCK = Lock()
DRONE_ACTUAL_POSITIONS = {
    "cf1": np.zeros(2),
    "cf2": np.zeros(2),
    "cf3": np.zeros(2),
    "cf4": np.zeros(2),
    "cf5": np.zeros(2),
    "cf6": np.zeros(2),
}


# def scale_to_raster_coordinates(x: float, y: float) -> Tuple[float, float]:
#    x = x + 100.0
#    y = y + 100.0
#    return x, y


# def scale_to_gui_coordinates(x: float, y: float) -> Tuple[float, float]:
#    x = x - 100.0
#    y = y - 100.0
#    return x, y


def solve_central_mass_point(drone: Drone, environment) -> Optional[Tuple[float, float]]:
    assert isinstance(drone, Drone)
    points: List[sh.Point] = list()
    with DRONE_ACTUAL_POSITIONS_LOCK:
        for _name, position in DRONE_ACTUAL_POSITIONS.items():
            x, y = position
            # TODO: scale to raster coordinates
            # x, y = scale_to_raster_coordinates(x, y)
            # x = x + 100.0
            # y = y + 100.0

            point = sh.Point(x, y)
            if point.is_empty:
                print(f"[{drone.name}] point is empty")
                return None
            points.append(point)
    multipoint = sh.MultiPoint(points)
    diagram = voronoi_diagram(multipoint)
    assert isinstance(diagram, sh.GeometryCollection)
    if len(list(diagram.geoms)) <= 0:
        print(f"[{drone.name}] voronoi diagram geometry collection is empty")
        print(f"  - {points}")
        return None
    with DRONE_ACTUAL_POSITIONS_LOCK:
        xi, yi = DRONE_ACTUAL_POSITIONS[drone.name]
        # TODO:
        # xi, yi = scale_to_raster_coordinates(xi, yi)
    pi = sh.Point(xi, yi)
    polygon_i = None
    for geometry in diagram.geoms:
        # Clip to environment bounds
        polygon = ENVIRONMENT_POLYGON.intersection(geometry)
        assert isinstance(polygon, sh.Polygon)
        if polygon.contains(pi):
            polygon_i = polygon
            break
    if polygon_i is None:
        print(f"[{drone.name}] polygon i is none")
        return None
    # TODO: dimensions
    raster = rasterize([polygon_i], (TILES_IN_X, TILES_IN_Y)).astype(bool)
    # if not np.any(raster != 0.0):
    if np.all(raster == 0.0):
        print(f"[{drone.name}] non-zero raster not found")
        plt.matshow(raster)
        plt.show(True)
        raise Exception()
        return None
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
    EPSILON = 0.0001
    if total_mass < EPSILON:
        print(f"[{drone.name}] voronoi region total mass was zero")
        return None
    else:
        center_of_mass = upper / total_mass
        x, y = center_of_mass
        # Scale back to GUI coordinates
        # x, y = scale_to_gui_coordinates(x, y)
        return x, y
        # x = x - 100.0
        # y = y - 100.0
        # return (x, y)

        # return center_of_mass
        # TODO: super_polygons[i] = polygon_i


def control_thread(drone: Drone):
    assert isinstance(drone, Drone)
    print(f"drone {drone.name} control start")
    tolerance = 0.1
    delta_time = 0.1

    with DRONE_ACTUAL_POSITIONS_LOCK:
        # TODO: support z and angle
        x0, y0 = DRONE_ACTUAL_POSITIONS[drone.name]

    do_continue = True
    setpoint_x = x0
    setpoint_y = y0
    autopilot = False
    environment = None
    while do_continue:
        if 0 < drone.queue.qsize():
            command = drone.queue.get()
            if isinstance(command, CommandQuit):
                do_continue = False
            elif isinstance(command, CommandState):
                with DRONE_ACTUAL_POSITIONS_LOCK:
                    x0, y0 = DRONE_ACTUAL_POSITIONS[drone.name]
                print(f"drone {drone.name} is at ({x0}, {y0})")
            elif isinstance(command, CommandStop):
                with DRONE_ACTUAL_POSITIONS_LOCK:
                    x0, y0 = DRONE_ACTUAL_POSITIONS[drone.name]
                setpoint_x = x0
                setpoint_y = y0
            elif isinstance(command, CommandMove):
                match command.move_type:
                    case MoveType.ABSOLUTE:
                        setpoint_x = command.x
                        setpoint_y = command.y
                    case MoveType.RELATIVE:
                        with DRONE_ACTUAL_POSITIONS_LOCK:
                            x0, y0 = DRONE_ACTUAL_POSITIONS[drone.name]
                        setpoint_x = x0 + command.x
                        setpoint_y = y0 + command.y
            elif isinstance(command, CommandAutopilot):
                autopilot = command.state
            elif isinstance(command, CommandUpdateEnvironment):
                environment = deepcopy(command.environment)
            else:
                print(f"drone {drone.name} received unknown command: {command}")
        else:
            pass
        with DRONE_ACTUAL_POSITIONS_LOCK:
            x0, y0 = DRONE_ACTUAL_POSITIONS[drone.name]
        if autopilot:
            # Ignore user's setpoints
            if environment is None:
                print(f"drone {drone.name} has no environment")
                setpoint_x = x0
                setpoint_y = y0
            else:
                result = solve_central_mass_point(drone, environment)
                if result is None:
                    setpoint_x = x0  # np.random.uniform(-100.0, 100.0)
                    setpoint_y = y0  # np.random.uniform(-100.0, 100.0)
                else:
                    # print(result)
                    x, y = result
                    setpoint_x = x
                    setpoint_y = y
                # print(f"drone {drone.name} new target: ({setpoint_x}, {setpoint_y})")
        else:
            pass

        dx = setpoint_x - x0
        dy = setpoint_y - y0
        distance = np.sqrt((dx**2) + (dy**2))
        if tolerance < abs(distance):
            direction_x = dx / distance
            direction_y = dy / distance
            speed = min(drone.maximum_speed, distance)
            translation_x = (speed * delta_time) * direction_x
            translation_y = (speed * delta_time) * direction_y
            with DRONE_ACTUAL_POSITIONS_LOCK:
                DRONE_ACTUAL_POSITIONS[drone.name][0] += translation_x
                DRONE_ACTUAL_POSITIONS[drone.name][1] += translation_y
        time.sleep(delta_time)
    print(f"drone {drone.name} finished")


class GraphicsTileItem(QGraphicsRectItem):
    def __init__(self, x: float, y: float, z: float):
        assert isinstance(x, float)
        assert isinstance(y, float)
        assert isinstance(z, float)
        self.rectangle = QRectF(x, y, PIXELS_PER_TILE, PIXELS_PER_TILE)
        super().__init__(self.rectangle)
        self.value = z
        green = int(self.value * 255)
        self.setPen(QColor(0, green, 0))
        self.setBrush(QColor(0, green, 0))


class GraphicsDroneItem(QGraphicsRectItem):
    def __init__(self, x: float, y: float):
        assert isinstance(x, float)
        assert isinstance(y, float)
        self.rectangle = QRectF(x, y, 10, 10)
        super().__init__(self.rectangle)
        self.setPen(QColor(0, 0, 255))
        self.setBrush(QColor(0, 0, 255))
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)

    def paint(self, painter: QPainter, option: QStyleOptionGraphicsItem, widget: Optional[QWidget] = None):
        # TODO: why color does not change???
        if self.isSelected():
            painter.setBrush(QColor(255, 255, 0))
        else:
            painter.setBrush(QColor(0, 0, 255))
        super().paint(painter, option, widget)


class GraphicsView(QGraphicsView):
    def __init__(self, scene: QGraphicsScene, parent: "MainWidget"):
        assert isinstance(scene, QGraphicsScene)
        assert isinstance(parent, MainWidget)
        super().__init__(scene=scene, parent=parent)

        self.main_widget = parent
        self.setBackgroundBrush(QColor(255, 255, 255))
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.setDragMode(QGraphicsView.DragMode.RubberBandDrag)
        self.setSceneRect(VIEW_RECTANGLE)
        self.setMouseTracking(True)
        self.zoom = 0

        self.rubberBandChanged.connect(self.rubber_band_changed)

    def mousePressEvent(self, event: QMouseEvent) -> None:
        match event.button():
            case Qt.MouseButton.LeftButton:
                super().mousePressEvent(event)
            case Qt.MouseButton.MiddleButton:
                super().mousePressEvent(event)
            case Qt.MouseButton.RightButton:
                target = self.mapToScene(event.pos())
                for connection in self.main_widget.selected_drones():
                    x = target.x()
                    y = target.y()
                    command = CommandMove(MoveType.ABSOLUTE, x, y)
                    self.main_widget.connected_drones[connection.name].queue.put(command)
            case _:
                pass

    def mouseMoveEvent(self, event: QMouseEvent):
        # Simulation position
        mps = self.mapToScene(event.pos())
        # Physical position
        xp, yp = simulation_to_physical(mps.x(), mps.y())
        self.main_widget.mouse_position_label.setText(f"({xp}, {yp})  ({mps.x()}, {mps.y()})")
        super().mouseMoveEvent(event)

    def wheelEvent(self, event: QWheelEvent) -> None:
        if 0 < event.angleDelta().y():
            self.scale(1.1, 1.1)
        else:
            self.scale(0.9, 0.9)
        super().wheelEvent(event)

    @Slot()
    def rubber_band_changed(self, viewportrect: QRect, fromscenepoint: QPointF, toscenepoint: QPointF):
        scene_rect = self.mapToScene(viewportrect).boundingRect()
        if scene_rect.isEmpty():
            # Do not clear selections
            pass
        else:
            painter_path = QPainterPath()
            painter_path.addRect(scene_rect)
            self.scene().setSelectionArea(painter_path)


class MainWidget(QWidget):
    def __init__(self):
        super().__init__()

        self.layout = QGridLayout(self)

        self.mouse_position_label = QLabel(self)
        self.mouse_position_label.setText(f"(?, ?)")
        self.layout.addWidget(self.mouse_position_label, 0, 1)

        self.view = self.make_view()
        self.layout.addWidget(self.view, 1, 1)

        console_layout = self.make_console()
        self.layout.addLayout(console_layout, 2, 0, 1, 2)

        self.selection_label = QLabel(self)
        self.layout.addWidget(self.selection_label, 3, 0, 1, 2)

        self.connected_drones: Dict[str, Drone] = dict()

        self.environment = self.make_environment()

        self.timer = QTimer(self)
        self.timer.setSingleShot(False)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.timer_timeout)
        self.timer.start()

    def make_environment(self):
        environment = create_environment(
            width=TILES_IN_PHYSICAL_X,
            height=TILES_IN_PHYSICAL_Y,
        )
        for i in range(TILES_IN_PHYSICAL_X):
            offset_tile_x = i
            offset_pixel_x = offset_tile_x * PIXELS_PER_TILE
            for j in range(TILES_IN_PHYSICAL_Y):
                offset_tile_y = j
                offset_pixel_y = offset_tile_y * PIXELS_PER_TILE

                z = environment[int(offset_tile_y), int(offset_tile_x)]
                z = float(z)

                tile = GraphicsTileItem(float(offset_pixel_x), float(offset_pixel_y), z)
                self.view.scene().addItem(tile)
        return environment

    def make_view(self) -> GraphicsView:
        scene = QGraphicsScene(self)
        scene.addRect(LABORATORY_SIMULATION_RECTANGLE)
        view = GraphicsView(scene=scene, parent=self)
        return view

    def selected_drones(self) -> List[Drone]:
        selections: List[Drone] = list()
        for item in self.view.scene().selectedItems():
            for _name, drone in self.connected_drones.items():
                if drone.graphics_item is item:
                    selections.append(drone)
        return selections

    def timer_timeout(self):
        # Print information about selected drones
        drones: List[str] = list()
        for drone in self.selected_drones():
            with DRONE_ACTUAL_POSITIONS_LOCK:
                x0, y0 = DRONE_ACTUAL_POSITIONS[drone.name]
            drones.append(f"{drone.name} ({x0}, {y0})")
        drones_str = "\n".join(drones)
        self.selection_label.setText(drones_str)

        # Update graphics item position
        for _i, (name, drone) in enumerate(self.connected_drones.items()):
            with DRONE_ACTUAL_POSITIONS_LOCK:
                x0, y0 = deepcopy(DRONE_ACTUAL_POSITIONS[name])
            if drone.graphics_item is None:
                drone.graphics_item = GraphicsDroneItem(x=x0, y=y0)
                self.view.scene().addItem(drone.graphics_item)
            else:
                position = QPointF(x0, y0)
                drone.graphics_item.setPos(position)

    def closeEvent(self, event: QCloseEvent):
        if 0 < len(self.connected_drones):
            while 0 < len(self.connected_drones):
                _name, connection = self.connected_drones.popitem()
                connection.queue.put(CommandQuit())
        # event.accept()
        super().closeEvent(event)

    def make_completer(self):
        words = [
            "help",
            "quit",
            "connections",
            "connect",
            "disconnect",
            # TODO
            # "connect 1",
            # "connect 2",
            # "connect 3",
            # "connect 4",
            "move",
            "move absolute",
            "move relative",
            "state",
            "stop",
            "autopilot",
            "autopilot on",
            "autopilot off",
            "update",
            "update environment",
        ]
        completer = QCompleter(words, self)
        # TODO: set case sensitivity?
        return completer

    def make_console(self):
        layout = QGridLayout(self)

        font = QFont("Monospace")
        font.setStyleHint(QFont.StyleHint.TypeWriter)

        self.console_input = QLineEdit(self)
        self.console_input.setFont(font)
        self.console_input.setPlaceholderText(f"command")
        completer = self.make_completer()
        self.console_input.setCompleter(completer)
        self.console_input.returnPressed.connect(self.console_input_enter_pressed)
        layout.addWidget(self.console_input, 1, 0)

        return layout

    def print_help(self):
        print(f"  [help] print this message")
        print(f"  [quit] stop operation")
        print(f"  [connections] print established connections")
        print(f"  [connect <number>] establish connection to drone")
        print(f"  [disconnect <number>] break connection to drone")
        print(f"  [state <number>] print drone's state")
        print(f"  [stop <number>] stop drone")
        print(f"  [move relative <number> <x> <y>]")
        print(f"  [move absolute <number> <x> <y>]")
        print(f"  [autopilot <on/off>]")

    def print_connections(self):
        for i, (name, drone) in enumerate(self.connected_drones.items()):
            print(f"  {i + 1}: {name}, {drone.uri}")

    def try_parse_drone_name(self, text: str) -> Optional[str]:
        assert isinstance(text, str)
        try:
            number = int(text)
            name = f"cf{number}"
            if name in NAME_TO_URI.keys():
                return name
            else:
                print(f"drone {name} does not exist")
                return None
        except ValueError as error:
            print(f"could not parse drone name: {error}")
            return None

    def connect_drone(self, name: str):
        assert isinstance(name, str)
        if name in self.connected_drones.keys():
            print(f"connection to drone {name} is already established")
        else:
            uri = NAME_TO_URI[name]
            queue: Queue[Command] = Queue()
            drone = Drone(name, uri, queue)
            Thread(target=control_thread, args=[drone]).start()
            self.connected_drones[name] = drone

    def disconnect_drone(self, name: str):
        assert isinstance(name, str)
        if name in self.connected_drones.keys():
            connection = self.connected_drones.pop(name)
            connection.queue.put(CommandQuit())
            if connection.graphics_item is not None:
                self.view.scene().removeItem(connection.graphics_item)
        else:
            print(f"not connected to drone {name}")

    def evaluate_command(self, command: str):
        assert isinstance(command, str)
        parts = command.split(" ")
        if 0 < len(parts):
            match parts[0]:
                case "help":
                    self.print_help()
                case "quit":
                    self.close()
                case "connections":
                    self.print_connections()
                case "connect":
                    match self.try_parse_drone_name(parts[1]):
                        case None:
                            pass
                        case name:
                            self.connect_drone(name)
                case "disconnect":
                    match self.try_parse_drone_name(parts[1]):
                        case None:
                            pass
                        case name:
                            self.disconnect_drone(name)
                case "move":
                    match parts[1]:
                        case "absolute" | "relative":
                            match self.try_parse_drone_name(parts[2]):
                                case None:
                                    pass
                                case name:
                                    try:
                                        x = float(parts[3])
                                        y = float(parts[4])
                                        if name in self.connected_drones.keys():
                                            match parts[1]:
                                                case "absolute":
                                                    move_type = MoveType.ABSOLUTE
                                                case "relative":
                                                    move_type = MoveType.RELATIVE
                                                case other:
                                                    print(f"unknown move type: {other}")
                                                    move_type = None
                                            if move_type is not None:
                                                self.connected_drones[name].queue.put(CommandMove(move_type, x, y))
                                        else:
                                            print(f"not connected to drone {name}")
                                    except ValueError as error:
                                        print(f"could not parse target coordinates: {error}")
                        case other:
                            print(f"unknown command: {other}")
                case "state":
                    match self.try_parse_drone_name(parts[1]):
                        case None:
                            pass
                        case name:
                            if name in self.connected_drones.keys():
                                drone = self.connected_drones[name]
                                drone.queue.put(CommandState())
                            else:
                                print(f"not connected to drone {name}")
                case "stop":
                    match self.try_parse_drone_name(parts[1]):
                        case None:
                            pass
                        case name:
                            if name in self.connected_drones.keys():
                                drone = self.connected_drones[name]
                                drone.queue.put(CommandStop())
                            else:
                                print(f"not connected to drone {name}")
                case "autopilot":
                    match parts[1]:
                        case "on" | "off":
                            if parts[1] == "on":
                                state = True
                            else:
                                state = False
                            for _name, drone in self.connected_drones.items():
                                drone.queue.put(CommandAutopilot(state))
                        case other:
                            print(f"unknown command: {other}")
                case "update":
                    match parts[1]:
                        case "environment":
                            for _name, drone in self.connected_drones.items():
                                environment = deepcopy(self.environment)
                                drone.queue.put(CommandUpdateEnvironment(environment))
                        case other:
                            print(f"unknown command: {other}")
                case other:
                    print(f"unknown command: {other}")
        else:
            pass

    @Slot()
    def console_input_enter_pressed(self):
        command = self.console_input.text()
        self.console_input.clear()
        self.evaluate_command(command)


if __name__ == "__main__":
    app = QApplication([])
    widget = MainWidget()
    widget.resize(800, 600)
    widget.show()
    sys.exit(app.exec())
