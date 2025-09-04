import math

from pysimtorcs.geometry import Vector2
from pysimtorcs.segments import Segment


class Grid:
    def __init__(self, track_width: float, track_height: float, cell_size: float):
        self.cell_size = cell_size
        self.grid_width = math.ceil(track_width / cell_size)
        self.grid_height = math.ceil(track_height / cell_size)

        # Erstelle ein 2D-Array (Liste von Listen) für das Raster
        self.cells: list[list[list[Segment]]] = [
            [[] for _ in range(self.grid_height)] for _ in range(self.grid_width)
        ]

    def get_cell_coords(self, x: float, y: float) -> tuple[int, int]:
        """
        Berechnet die Koordinaten der Rasterzelle für eine gegebene Position.
        """
        cell_x = int(x / self.cell_size)
        cell_y = int(y / self.cell_size)

        # Stelle sicher, dass die Koordinaten innerhalb der Rastergrenzen liegen
        cell_x = max(0, min(cell_x, self.grid_width - 1))
        cell_y = max(0, min(cell_y, self.grid_height - 1))

        return cell_x, cell_y

    def add_segment(self, segment: Segment) -> None:
        """
        Fügt ein Segment dem Raster hinzu.
        Es wird in jede Zelle eingetragen, die seine Bounding Box berührt.
        """

        min_x, min_y, max_x, max_y = segment.get_bbox()

        # Berechne den Start- und End-Zellbereich
        start_cell_x, start_cell_y = self.get_cell_coords(min_x, min_y)
        end_cell_x, end_cell_y = self.get_cell_coords(max_x, max_y)

        for x in range(start_cell_x, end_cell_x + 1):
            for y in range(start_cell_y, end_cell_y + 1):
                self.cells[x][y].append(segment)

    def get_segments_at_position(self, position: Vector2) -> list[Segment]:
        """
        Gibt eine Liste aller Segmente in der Zelle der gegebenen Position zurück.
        """
        cell_x, cell_y = self.get_cell_coords(position.x, position.y)

        # Da ein Auto sich schnell bewegt, ist es sicherer, auch die
        # 8 Nachbarzellen zu überprüfen, um sicherzugehen, dass
        # keine Zellgrenze "übersprungen" wird.
        candidate_segments = set()
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                check_x, check_y = cell_x + dx, cell_y + dy
                if 0 <= check_x < self.grid_width and 0 <= check_y < self.grid_height:
                    for segment in self.cells[check_x][check_y]:
                        candidate_segments.add(segment)

        return list(candidate_segments)
