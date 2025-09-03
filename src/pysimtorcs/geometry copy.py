import numpy as np


class Vector2(np.ndarray):
    """
    Eine 2D-Vektor-Klasse, implementiert als NumPy-Subklasse.
    """

    def __new__(cls, x, y):
        """
        Der __new__-Konstruktor erstellt das NumPy-Array-Objekt.
        """
        # Wir müssen ein Array mit zwei Float-Werten erstellen
        # und die `dtype` sowie die `shape` festlegen.
        obj = np.asarray([x, y], dtype=np.float64).view(cls)
        return obj

    def __init__(self, x, y):
        # __init__ wird nur für die Initialisierung von Werten aufgerufen.
        # Da wir das Array bereits in __new__ initialisiert haben,
        # brauchen wir hier nichts zu tun, außer der Lesbarkeit halber.
        pass

    # Eigenschaften für den bequemen Zugriff auf die Vektor-Komponenten
    @property
    def x(self):
        return self[0]

    @x.setter
    def x(self, value):
        self[0] = value

    @property
    def y(self):
        return self[1]

    @y.setter
    def y(self, value):
        self[1] = value

    def __str__(self):
        return f"Vector2({self.x}, {self.y})"

    def __repr__(self):
        return f"Vector2({self.x}, {self.y})"

    def length(self):
        """Gibt die Länge (Magnitude) des Vektors zurück."""
        return np.linalg.norm(self)

    def normalize(self):
        """Gibt einen normalisierten Vektor zurück (Länge = 1)."""
        length = self.length()
        if length == 0:
            return Vector2(0, 0)
        return self / length
