#!/usr/bin/env python3

import heapq
import math
import os

import yaml


def _read_pgm_token(file_handle):
    token = bytearray()
    while True:
        char = file_handle.read(1)
        if not char:
            return bytes(token)
        if char == b"#":
            file_handle.readline()
            continue
        if char.isspace():
            if token:
                return bytes(token)
            continue
        token.extend(char)


def load_pgm_image(path):
    with open(path, "rb") as file_handle:
        magic = _read_pgm_token(file_handle).decode("ascii")
        width = int(_read_pgm_token(file_handle))
        height = int(_read_pgm_token(file_handle))
        max_value = int(_read_pgm_token(file_handle))

        if magic == "P5":
            if max_value > 255:
                raise ValueError("Only 8-bit PGM images are supported: %s" % path)
            pixel_bytes = file_handle.read(width * height)
            if len(pixel_bytes) != width * height:
                raise ValueError("Unexpected PGM payload size in %s" % path)
            pixels = list(pixel_bytes)
        elif magic == "P2":
            pixels = []
            for _ in range(width * height):
                token = _read_pgm_token(file_handle)
                if not token:
                    raise ValueError("Unexpected end of ASCII PGM payload in %s" % path)
                pixels.append(int(token))
        else:
            raise ValueError("Unsupported PGM format %s in %s" % (magic, path))

    return width, height, max_value, pixels


class StaticOccupancyMap:
    def __init__(
        self,
        width,
        height,
        resolution,
        origin_x,
        origin_y,
        traversable,
    ):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.traversable = traversable
        self.neighbor_steps = [
            (-1, 0, resolution),
            (1, 0, resolution),
            (0, -1, resolution),
            (0, 1, resolution),
            (-1, -1, resolution * math.sqrt(2.0)),
            (-1, 1, resolution * math.sqrt(2.0)),
            (1, -1, resolution * math.sqrt(2.0)),
            (1, 1, resolution * math.sqrt(2.0)),
        ]

    @classmethod
    def from_map_yaml(cls, yaml_path, clearance_m=0.0, allow_unknown=False):
        with open(yaml_path, "r") as file_handle:
            map_config = yaml.safe_load(file_handle) or {}

        image_path = cls.resolve_image_path(yaml_path, map_config.get("image"))
        width, height, max_value, pixels = load_pgm_image(image_path)

        resolution = float(map_config["resolution"])
        origin = map_config.get("origin", [0.0, 0.0, 0.0])
        origin_x = float(origin[0])
        origin_y = float(origin[1])
        negate = int(map_config.get("negate", 0))
        occupied_thresh = float(map_config.get("occupied_thresh", 0.65))
        free_thresh = float(map_config.get("free_thresh", 0.196))

        blocked = [False] * (width * height)
        for index, pixel_value in enumerate(pixels):
            normalized = float(pixel_value) / float(max_value)
            occupancy = normalized if negate else 1.0 - normalized
            is_occupied = occupancy >= occupied_thresh
            is_unknown = free_thresh < occupancy < occupied_thresh
            blocked[index] = is_occupied or (is_unknown and not allow_unknown)

        clearance_cells = int(math.ceil(max(0.0, clearance_m) / resolution))
        if clearance_cells > 0:
            blocked = cls.inflate_blocked_cells(blocked, width, height, clearance_cells)

        traversable = [not value for value in blocked]
        return cls(width, height, resolution, origin_x, origin_y, traversable)

    @staticmethod
    def resolve_image_path(yaml_path, image_reference):
        if not image_reference:
            raise ValueError("Map yaml does not define an image path: %s" % yaml_path)

        image_reference = os.path.expanduser(str(image_reference))
        yaml_directory = os.path.dirname(os.path.realpath(yaml_path))
        candidates = [
            image_reference,
            os.path.join(yaml_directory, image_reference),
            os.path.join(yaml_directory, os.path.basename(image_reference)),
        ]

        for candidate in candidates:
            if os.path.exists(candidate):
                return candidate

        raise FileNotFoundError(
            "Could not resolve map image %s from %s" % (image_reference, yaml_path)
        )

    @staticmethod
    def inflate_blocked_cells(blocked, width, height, radius_cells):
        offsets = []
        for row_offset in range(-radius_cells, radius_cells + 1):
            for col_offset in range(-radius_cells, radius_cells + 1):
                if row_offset * row_offset + col_offset * col_offset <= radius_cells * radius_cells:
                    offsets.append((row_offset, col_offset))

        inflated = list(blocked)
        blocked_indices = [index for index, value in enumerate(blocked) if value]
        for index in blocked_indices:
            row = index // width
            col = index % width
            for row_offset, col_offset in offsets:
                next_row = row + row_offset
                next_col = col + col_offset
                if 0 <= next_row < height and 0 <= next_col < width:
                    inflated[next_row * width + next_col] = True

        return inflated

    def world_to_grid(self, x, y):
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row_from_bottom = int(math.floor((y - self.origin_y) / self.resolution))
        row = self.height - 1 - row_from_bottom

        if row < 0 or row >= self.height or col < 0 or col >= self.width:
            return None
        return row, col

    def grid_to_world(self, row, col):
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (self.height - row - 0.5) * self.resolution
        return x, y

    def grid_index(self, row, col):
        return row * self.width + col

    def is_traversable(self, row, col):
        return self.traversable[self.grid_index(row, col)]

    def snap_to_free(self, cell, max_radius_cells=4):
        if cell is None:
            return None

        row, col = cell
        if self.is_traversable(row, col):
            return row, col

        best_cell = None
        best_distance = None
        for radius in range(1, max_radius_cells + 1):
            for row_offset in range(-radius, radius + 1):
                for col_offset in range(-radius, radius + 1):
                    if max(abs(row_offset), abs(col_offset)) != radius:
                        continue

                    next_row = row + row_offset
                    next_col = col + col_offset
                    if next_row < 0 or next_row >= self.height or next_col < 0 or next_col >= self.width:
                        continue
                    if not self.is_traversable(next_row, next_col):
                        continue

                    distance = math.hypot(row_offset, col_offset)
                    if best_distance is None or distance < best_distance:
                        best_distance = distance
                        best_cell = (next_row, next_col)

            if best_cell is not None:
                return best_cell

        return None

    def heuristic_cost(self, source_cell, target_cell):
        row_delta = source_cell[0] - target_cell[0]
        col_delta = source_cell[1] - target_cell[1]
        return math.hypot(row_delta, col_delta) * self.resolution

    def plan_path(self, start_x, start_y, goal_x, goal_y):
        start_cell = self.snap_to_free(self.world_to_grid(start_x, start_y))
        goal_cell = self.snap_to_free(self.world_to_grid(goal_x, goal_y))
        if start_cell is None or goal_cell is None:
            return float("inf"), []

        start_index = self.grid_index(*start_cell)
        goal_index = self.grid_index(*goal_cell)

        queue = [(self.heuristic_cost(start_cell, goal_cell), 0.0, start_index)]
        best_costs = {start_index: 0.0}
        parents = {start_index: None}

        while queue:
            _, current_cost, current_index = heapq.heappop(queue)
            if current_index == goal_index:
                break
            if current_cost > best_costs.get(current_index, float("inf")):
                continue

            current_row = current_index // self.width
            current_col = current_index % self.width
            for row_offset, col_offset, step_cost in self.neighbor_steps:
                next_row = current_row + row_offset
                next_col = current_col + col_offset
                if next_row < 0 or next_row >= self.height or next_col < 0 or next_col >= self.width:
                    continue
                if not self.is_traversable(next_row, next_col):
                    continue

                next_index = self.grid_index(next_row, next_col)
                next_cost = current_cost + step_cost
                if next_cost >= best_costs.get(next_index, float("inf")):
                    continue

                best_costs[next_index] = next_cost
                parents[next_index] = current_index
                heuristic = self.heuristic_cost((next_row, next_col), goal_cell)
                heapq.heappush(queue, (next_cost + heuristic, next_cost, next_index))

        if goal_index not in parents:
            return float("inf"), []

        grid_path = []
        current_index = goal_index
        while current_index is not None:
            current_row = current_index // self.width
            current_col = current_index % self.width
            grid_path.append((current_row, current_col))
            current_index = parents[current_index]
        grid_path.reverse()

        world_path = [(start_x, start_y)]
        for row, col in grid_path[1:-1]:
            world_path.append(self.grid_to_world(row, col))
        world_path.append((goal_x, goal_y))

        total_cost = best_costs[goal_index]
        return total_cost, world_path
