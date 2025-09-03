# import numpy as np


# class Vector2(np.ndarray):
#     """
#     Eine 2D-Vektor-Klasse, implementiert als NumPy-Subklasse.
#     """

#     def __new__(cls, x: float = 0, y: float = 0):
#         """
#         Der __new__-Konstruktor erstellt das NumPy-Array-Objekt.
#         """
#         # Wir müssen ein Array mit zwei Float-Werten erstellen
#         # und die `dtype` sowie die `shape` festlegen.
#         obj = np.asarray([x, y], dtype=np.float64).view(cls)
#         return obj

#     def __init__(self, x: float = 0, y: float = 0):
#         # __init__ wird nur für die Initialisierung von Werten aufgerufen.
#         # Da wir das Array bereits in __new__ initialisiert haben,
#         # brauchen wir hier nichts zu tun, außer der Lesbarkeit halber.
#         pass

#     # Eigenschaften für den bequemen Zugriff auf die Vektor-Komponenten
#     @property
#     def x(self):
#         return self[0]

#     @x.setter
#     def x(self, value):
#         self[0] = value

#     @property
#     def y(self):
#         return self[1]

#     @y.setter
#     def y(self, value):
#         self[1] = value

#     def __str__(self):
#         return f"Vector2({self.x}, {self.y})"

#     def __repr__(self):
#         return f"Vector2({self.x}, {self.y})"

#     def length(self):
#         """Gibt die Länge (Magnitude) des Vektors zurück."""
#         return np.linalg.norm(self)

#     def normalize(self):
#         """Gibt einen normalisierten Vektor zurück (Länge = 1)."""
#         length = self.length()
#         if length == 0:
#             return Vector2(0, 0)
#         return self / length

#     def cross(self, other: "Vector2") -> float:
#         return self.x * other.y - self.y * other.x

#     def scale(self, scalar: float) -> "Vector2":
#         return self * scalar

#     def rotate(self, angle_in_rad: float) -> "Vector2":
#         cos_angle = np.cos(angle_in_rad)
#         sin_angle = np.sin(angle_in_rad)
#         return Vector2(
#             x=self.x * cos_angle - self.y * sin_angle,
#             y=self.x * sin_angle + self.y * cos_angle,
#         )

#     def distance(self, other: "Vector2") -> float:
#         delta_vector = self - other
#         return delta_vector.length()


# # v1 = Vector2(3, 4)
# # v2 = Vector2(1, 2)

# # v3 = v1 + v2
# # print(v3)
# # v4 = v1 - v2
# # print(v4)
# # v5 = v1 * v2
# # print(v5)
# # v6 = v1 / v2
# # print(v6)


# # v7 = v1.scale(4)
# # print(v7)
# # v8 = v7.cross(v2)
# # print(v8)
