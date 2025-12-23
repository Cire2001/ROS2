from exercice3.vec2d import Vec2D

class Map2D:
    def __init__(self, _map):
        self.map = _map
        self.width = _map.info.width
        self.height = _map.info.height
        self.resolution = _map.info.resolution
        self.origin_x = _map.info.origin.position.x
        self.origin_y = _map.info.origin.position.y
        self.data = _map.data

    def convert_coords(self, pos):
        """Convert world coordinates to grid index"""
        x_index = int((pos.x - self.origin_x) / self.resolution)
        y_index = int((pos.y - self.origin_y) / self.resolution)

        # Calculate index in the data array
        index = y_index * self.width + x_index
        return index

    def is_wall(self, pos, tolerance=0.0):
        """Check if position is occupied, with tolerance parameter"""
        # Convert to grid coordinates
        x_index = int((pos.x - self.origin_x) / self.resolution)
        y_index = int((pos.y - self.origin_y) / self.resolution)

        # Check if position is within map bounds
        if x_index < 0 or x_index >= self.width or y_index < 0 or y_index >= self.height:
            return True

        # If tolerance is 0, just check the exact position
        if tolerance <= 0:
            index = y_index * self.width + x_index
            return self.data[index] > 0.65

        # With tolerance, check surrounding area
        tolerance_cells = max(1, int(tolerance / self.resolution))

        # Check if any free space within tolerance radius
        for dy in range(-tolerance_cells, tolerance_cells + 1):
            for dx in range(-tolerance_cells, tolerance_cells + 1):
                check_x = x_index + dx
                check_y = y_index + dy

                # Check bounds
                if check_x < 0 or check_x >= self.width or check_y < 0 or check_y >= self.height:
                    continue

                # Check if this cell is free
                index = check_y * self.width + check_x
                if self.data[index] < 0.65:
                    return False

        # All cells within tolerance are occupied
        return True

    def in_bounds(self, pos):
        """Check if position is within map bounds"""
        x_index = int((pos.x - self.origin_x) / self.resolution)
        y_index = int((pos.y - self.origin_y) / self.resolution)

        return (0 <= x_index < self.width and 0 <= y_index < self.height)

    def get_adjacent(self, pos):
        """Get adjacent positions (8-connected) that are not walls"""
        adjacent_positions = []

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue

                new_pos = Vec2D(pos.x + dx * self.resolution, pos.y + dy * self.resolution)

                if self.in_bounds(new_pos) and not self.is_wall(new_pos):
                    adjacent_positions.append(new_pos)

        return adjacent_positions

    def euclidean_distance(self, a, b):
        """Calculate Euclidean distance between two points"""
        dx = a.x - b.x
        dy = a.y - b.y
        return (dx * dx + dy * dy) ** 0.5
