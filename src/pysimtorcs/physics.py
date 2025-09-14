from functools import cached_property
import xml.etree.ElementTree as ET
import math

from pydantic import BaseModel
from pygame import Vector2
from pygame.math import clamp
#
# Unterklasse für die Bremseigenschaften
class Brake(BaseModel):
    disk_diameter_mm: float
    piston_area_cm2: float
    mu: float
    inertia: float
    
# Unterklasse für die Radeigenschaften
class Wheel(BaseModel):
    ypos: float
    rim_diameter_in: float
    tire_width_mm: float
    tire_height_ratio: float
    inertia: float
    ride_height_mm: float
    stiffness: float
    camber_deg: float
    dynamic_friction: float
    mu: float
    
    @cached_property
    def radius_m(self) -> float:
        tire_height_mm = self.tire_width_mm * self.tire_height_ratio
        return (self.rim_diameter_in * 25.4 + 2 * tire_height_mm) / 2000

class CarPhysicsProperties(BaseModel):
    mass: float
    cg_height: float
    cg_to_front_axle: float
    cg_to_rear_axle: float
    steer_lock: float
    aero_cx: float
    aero_front_area: float

    front_clift: float
    rear_clift: float

    gear_ratios: dict[int, float]
    rear_diff_ratio: float
    rpm_limit: float
    rpm_tickover: float
    torque_curve: dict[int, tuple[float, float]]

    front_wheels: list[Wheel]
    rear_wheels: list[Wheel]
    front_brakes: list[Brake]
    rear_brakes: list[Brake]

    brake_max_pressure : float

    @cached_property
    def wheel_base(self) -> float:
        return self.cg_to_front_axle + self.cg_to_rear_axle

    @cached_property
    def axle_weight_ratio_front(self) -> float:
        return self.cg_to_rear_axle / self.wheel_base

    @cached_property
    def axle_weight_ratio_rear(self) -> float:
        return self.cg_to_front_axle / self.wheel_base

    @cached_property
    def wheel_radius(self) -> float:
        # Simplified: uses front right wheel radius
        return self.front_wheels[0].radius_m

    @cached_property
    def tire_grip(self) -> float:
        # Simplified: uses front right wheel mu
        return self.front_wheels[0].mu

    @cached_property
    def aero_drag_coefficient(self) -> float:
        return self.aero_cx * self.aero_front_area
    
    @cached_property
    def air_resist(self) -> float:
        air_density = 1.225 # kg/m^3
        return 0.5 * air_density * self.aero_drag_coefficient
    
    @cached_property
    def lift_constant(self) -> float:
        total_lift = (self.front_clift + self.rear_clift) 
        air_density = 1.225 # kg/m^3
        return 0.5 * air_density * total_lift * self.aero_front_area

class PhysicsEngineX:
    def __init__(self):
        self.gravity = 9.81  # m/s^2
        self.density_air = 1.225  # kg/m^3
    
    def update(self, car, dt : float):
        pass


import xml.etree.ElementTree as ET
import math
import bisect

class PhysicsEngine:
    def __init__(self, car_xml_file:str):
        """
        Initializes the physics engine by parsing car and track data.
        :param car_xml_file: Path to the car's XML file.
        :param track_xml_file: Path to the track's XML file.
        :param dt: Time step for simulation in seconds.
        """
        self.car_tree = ET.parse(car_xml_file)
        self.car_root = self.car_tree.getroot()

        front_wheels = [
            Wheel(ypos=self._get_car_value('Front Left Wheel', 'ypos'),
                  rim_diameter_in=self._get_car_value('Front Left Wheel', 'rim diameter'),
                  tire_width_mm=self._get_car_value('Front Left Wheel', 'tire width'),
                  tire_height_ratio=self._get_car_value('Front Left Wheel', 'tire height-width ratio'),
                  inertia=self._get_car_value('Front Left Wheel', 'inertia'),
                  ride_height_mm=self._get_car_value('Front Left Wheel', 'ride height'),
                  stiffness=self._get_car_value('Front Left Wheel', 'stiffness'),
                  camber_deg=self._get_car_value('Front Left Wheel', 'camber'),
                  dynamic_friction=self._get_car_value('Front Left Wheel', 'dynamic friction'),
                  mu=self._get_car_value('Front Left Wheel', 'mu')),
            Wheel(ypos=self._get_car_value('Front Right Wheel', 'ypos'),
                  rim_diameter_in=self._get_car_value('Front Right Wheel', 'rim diameter'),
                  tire_width_mm=self._get_car_value('Front Right Wheel', 'tire width'),
                  tire_height_ratio=self._get_car_value('Front Right Wheel', 'tire height-width ratio'),
                  inertia=self._get_car_value('Front Right Wheel', 'inertia'),
                  ride_height_mm=self._get_car_value('Front Right Wheel', 'ride height'),
                  stiffness=self._get_car_value('Front Right Wheel', 'stiffness'),
                  camber_deg=self._get_car_value('Front Right Wheel', 'camber'),
                  dynamic_friction=self._get_car_value('Front Right Wheel', 'dynamic friction'),
                  mu=self._get_car_value('Front Right Wheel', 'mu')),
        ]
        
        rear_wheels = [
            Wheel(ypos=self._get_car_value('Rear Left Wheel', 'ypos'),
                  rim_diameter_in=self._get_car_value('Rear Left Wheel', 'rim diameter'),
                  tire_width_mm=self._get_car_value('Rear Left Wheel', 'tire width'),
                  tire_height_ratio=self._get_car_value('Rear Left Wheel', 'tire height-width ratio'),
                  inertia=self._get_car_value('Rear Left Wheel', 'inertia'),
                  ride_height_mm=self._get_car_value('Rear Left Wheel', 'ride height'),
                  stiffness=self._get_car_value('Rear Left Wheel', 'stiffness'),
                  camber_deg=self._get_car_value('Rear Left Wheel', 'camber'),
                  dynamic_friction=self._get_car_value('Rear Left Wheel', 'dynamic friction'),
                  mu=self._get_car_value('Rear Left Wheel', 'mu')),
            Wheel(ypos=self._get_car_value('Rear Right Wheel', 'ypos'),
                  rim_diameter_in=self._get_car_value('Rear Right Wheel', 'rim diameter'),
                  tire_width_mm=self._get_car_value('Rear Right Wheel', 'tire width'),
                  tire_height_ratio=self._get_car_value('Rear Right Wheel', 'tire height-width ratio'),
                  inertia=self._get_car_value('Rear Right Wheel', 'inertia'),
                  ride_height_mm=self._get_car_value('Rear Right Wheel', 'ride height'),
                  stiffness=self._get_car_value('Rear Right Wheel', 'stiffness'),
                  camber_deg=self._get_car_value('Rear Right Wheel', 'camber'),
                  dynamic_friction=self._get_car_value('Rear Right Wheel', 'dynamic friction'),
                  mu=self._get_car_value('Rear Right Wheel', 'mu')),
        ]
        
        front_brakes = [
            Brake(disk_diameter_mm=self._get_car_value('Front Left Brake', 'disk diameter'),
                  piston_area_cm2=self._get_car_value('Front Left Brake', 'piston area'),
                  mu=self._get_car_value('Front Left Brake', 'mu'),
                  inertia=self._get_car_value('Front Left Brake', 'inertia')),
            Brake(disk_diameter_mm=self._get_car_value('Front Right Brake', 'disk diameter'),
                  piston_area_cm2=self._get_car_value('Front Right Brake', 'piston area'),
                  mu=self._get_car_value('Front Right Brake', 'mu'),
                  inertia=self._get_car_value('Front Right Brake', 'inertia')),
        ]

        rear_brakes = [
            Brake(disk_diameter_mm=self._get_car_value('Rear Left Brake', 'disk diameter'),
                  piston_area_cm2=self._get_car_value('Rear Left Brake', 'piston area'),
                  mu=self._get_car_value('Rear Left Brake', 'mu'),
                  inertia=self._get_car_value('Rear Left Brake', 'inertia')),
            Brake(disk_diameter_mm=self._get_car_value('Rear Right Brake', 'disk diameter'),
                  piston_area_cm2=self._get_car_value('Rear Right Brake', 'piston area'),
                  mu=self._get_car_value('Rear Right Brake', 'mu'),
                  inertia=self._get_car_value('Rear Right Brake', 'inertia')),
        ]
        
        # Get torque curve
        torque_curve_dict = {}
        data_points_section = self.car_root.find("section[@name='Engine']/section[@name='data points']")
        if data_points_section:
            for point in data_points_section.findall("section"):
                rpm = float(point.find("attnum[@name='rpm']").get("val"))
                tq = float(point.find("attnum[@name='Tq']").get("val"))
                torque_curve_dict[rpm] = tq

        # Get gear ratios
        gear_ratios_dict = {}
        gears_section = self.car_root.find("section[@name='Gearbox']/section[@name='gears']")
        if gears_section:
            for gear in gears_section.findall("section"):
                name = gear.get("name")
                # Exclude reverse gear for this simplified model
                if name != 'r':
                    ratio = float(gear.find("attnum[@name='ratio']").get("val"))
                    gear_ratios_dict[int(name)] = ratio

        # Create CarPhysicsProperties instance
        self.car_properties= CarPhysicsProperties(
            mass=self._get_car_value('Car', 'mass'),
            cg_height=self._get_car_value('Car', 'GC height'),
            cg_to_front_axle=self._get_car_value('Front Axle', 'xpos'),
            cg_to_rear_axle=abs(self._get_car_value('Rear Axle', 'xpos')),
            wheel_base=self._get_car_value('Front Axle', 'xpos') + abs(self._get_car_value('Rear Axle', 'xpos')),
            steer_lock=self._get_car_value('Steer', 'steer lock', default=21),
            aero_cx=self._get_car_value('Aerodynamics', 'Cx'),
            aero_front_area=self._get_car_value('Aerodynamics', 'front area'),
            front_clift=self._get_car_value('Aerodynamics', 'front Clift'),
            rear_clift=self._get_car_value('Aerodynamics', 'rear Clift'),
            rear_diff_ratio=self._get_car_value('Rear Differential', 'ratio'),
            rpm_tickover=self._get_car_value('Engine', 'tickover', default=900),
            rpm_limit=self._get_car_value('Engine', 'revs limiter', default=10000),
            gear_ratios=self.get_gear_ratios(),
            torque_curve=self.get_torque_curve(),
            front_wheels=front_wheels,
            rear_wheels=rear_wheels,
            front_brakes=front_brakes,
            rear_brakes=rear_brakes,
            brake_max_pressure=self._get_car_value('Brake System', 'max pressure'),
            tire_grip=self._get_car_value('Front Right Wheel', 'mu')
        )
        self.gear = 1
        self.velocity = Vector2(0,0)

    def calculate_forces(self, throttle=0.0, brake=0.0):
        """
        Calculates all forces acting on the car for the current time step.
        :param throttle: Throttle input (0.0 to 1.0)
        :param brake: Brake input (0.0 to 1.0)
        :return: Total longitudinal force in Newtons.
        """
        v = self.velocity.length()

        # 1. Drive force
        rpm = self.calculate_rpm(v, self.gear)
        # Simplified automatic gear shifting logic for demonstration
        gear_change = 0
        if self.gear < 6 and v > 0:
            if rpm >= 8000:
                gear_change = 1
        elif self.gear > 1 and v > 0:
            if rpm < 3000:
                gear_change = -1

        if gear_change != 0:
            self.gear += gear_change
            rpm = self.calculate_rpm(v, self.gear)

        # Interpolate torque from the torque curve.
        rpm_1k = rpm // 1000
        (m, a) = self.car_properties.torque_curve.get(rpm_1k)
        torque = m * rpm + a

        gear_ratio = self.car_properties.gear_ratios.get(self.gear, 0.0)
        diff_ratio = self.car_properties.rear_diff_ratio

        drive_force = torque * gear_ratio * diff_ratio / self.car_properties.wheel_radius
     
        v2 = v * abs(v)
        # 2. Aerodynamic forces (Lift and Drag)
        # Calculate lift force (N)
        lift_force = self.car_properties.lift_constant * v2
        
        # Calculate air drag force (N)
        air_drag_force = -self.car_properties.air_resist * v2

        # 3. Rolling resistance force (now a constant force, influenced by lift)
        # Static normal force (car's weight)
        normal_force_static = self.car_properties.mass * 9.81#self.car_properties.gravity
        # Dynamic normal force (reduced by lift at high speeds)
        normal_force_dynamic = normal_force_static - lift_force
        # Rolling resistance force uses the dynamic normal force
        rolling_resistance_force = 0.01 * normal_force_dynamic
        # rolling_resistance_force = -self._get_track_value("Surfaces/rroad", "rolling resistance") * normal_force_dynamic

        # 4. Total longitudinal force on the car

    
        # Simplified brake force
        brake = 0.1
        total_brake_force = (self.car_properties.front_brakes[0].mu * self.car_properties.front_brakes[0].piston_area_cm2 / 10000) * 2 # Front brakes
        total_brake_force += (self.car_properties.rear_brakes[0].mu * self.car_properties.rear_brakes[0].piston_area_cm2 / 10000) * 2 # Rear brakes
        
        total_force = (drive_force * throttle) + (total_brake_force * brake) + air_drag_force + rolling_resistance_force

        return total_force
    
    def update(self, throttle=0.0, brake=0.0):
        """
        Updates the car's state for one time step.
        :param throttle: Throttle input (0.0 to 1.0)
        :param brake: Brake input (0.0 to 1.0)
        """
        # Calculate forces
        total_force = self.calculate_forces(throttle, brake)

        # Calculate acceleration and update velocity/position
        acceleration = total_force / self.car_properties.mass
        self.velocity += acceleration * self.dt
        self.position += self.velocity * self.dt
        
    def calculate_rpm(self, velocity:float, gear:int)->int:
        if gear == 0 or self.car_properties.wheel_radius == 0:
            return self.car_properties.rpm_tickover
        
        wheel_rotation_rate = velocity / self.car_properties.wheel_radius
        gear_ratio = self.car_properties.gear_ratios.get(gear, 0.0)
        diff_ratio = self.car_properties.rear_diff_ratio
        
        rpm = wheel_rotation_rate * gear_ratio * diff_ratio * 60 / (2 * math.pi)

        return clamp(rpm, self.car_properties.rpm_tickover, self.car_properties.rpm_limit)        

    def run_simulation(self, steps, throttle_input, brake_input):
        """
        Runs a simplified simulation for a number of steps.
        :param steps: Number of simulation steps.
        :param throttle_input: A list of throttle values for each step.
        :param brake_input: A list of brake values for each step.
        """
        for i in range(steps):
            throttle = throttle_input[i] if i < len(throttle_input) else 0.0
            brake = brake_input[i] if i < len(brake_input) else 0.0
            
            self.update(throttle, brake)
            print(f"Time Step: {i*self.dt:.2f}s | Velocity: {self.velocity:.2f} m/s | Gear: {self.gear}")


    def _get_car_value(self, section, name, default=None, transform=None):
        """Helper to extract a value from the car's XML."""
        try:
            element = self.car_root.find(f"section[@name='{section}']/attnum[@name='{name}']")
            value = float(element.get("val"))
            if transform:
                return transform(value)
            return value
        except (AttributeError, ValueError):
            return default

    def _get_track_value(self, section, name, default=None, transform=None):
        """Helper to extract a value from the track's XML."""
        try:
            element = self.track_root.find(f"section[@name='{section}']/attnum[@name='{name}']")
            value = float(element.get("val"))
            if transform:
                return transform(value)
            return value
        except (AttributeError, ValueError):
            return default

    def get_torque_curve(self)->dict[int, tuple[float, float]]:
        """Extracts the torque curve (RPM to Torque) from the XML."""
        torque_data = {}
        data_points_section = self.car_root.find("section[@name='Engine']/section[@name='data points']")
        if data_points_section is None:
            return {}

        for point in data_points_section.findall("section"):
            rpm = int(point.find("attnum[@name='rpm']").get("val"))
            tq = float(point.find("attnum[@name='Tq']").get("val"))
            torque_data[rpm] = tq

        # Now create linear functions between points.
        sorted_rpms = sorted(torque_data.keys())
        linear_torque_data : dict[int, tuple[float, float]] = {}
        for i in range(len(sorted_rpms) - 1):
            # y = mx + b
            rpm_from = sorted_rpms[i]
            rpm_to = sorted_rpms[i + 1]
            tq1 = torque_data[rpm_from]
            tq2 = torque_data[rpm_to]
            # Linear interpolation: tq = m * rpm + b
            m = (tq2 - tq1) / (rpm_to - rpm_from)
            b = tq1 - m * rpm_from

            linear_torque_data[rpm_from//1000] = (m, b)

        return linear_torque_data

    def get_gear_ratios(self) -> dict[int, float]:
        """Extracts the gearbox ratios from the XML."""
        ratios = {}
        gears_section = self.car_root.find("section[@name='Gearbox']/section[@name='gears']")
        if gears_section is None:
            return {}
        for gear in gears_section.findall("section"):
            name = gear.get("name")
            index = int(name) if name.isdigit() else -1
            ratio = float(gear.find("attnum[@name='ratio']").get("val"))
            ratios[index] = ratio
        return ratios

    def extract_constants(self):
        """
        Extracts and calculates the relevant physical constants.
        """
        # Basic constants from XML

        # Constants from track XML
        self.physics_constants['CRR_ROAD'] = self._get_track_value("Surfaces/rroad", "rolling resistance")
        self.physics_constants['FRICTION_ROAD'] = self._get_track_value("Surfaces/rroad", "friction")


# Initialize the engine
engine = PhysicsEngine("input/cars/car1-trb1.xml")
print("Engine initialized with the following constants:")

engine.gear=3
engine.velocity = Vector2(50, 20)
engine.calculate_forces(0,0)
engine.gear=3
engine.velocity = Vector2(10, 30)
engine.calculate_forces(0,0)