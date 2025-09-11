from functools import cached_property
import xml.etree.ElementTree as ET
import math

from pydantic import BaseModel

class CarPhysicsProperties(BaseModel):
    mass : float
    cg_height : float
    cg_to_front_axle : float
    cg_to_rear_axle : float
    tire_grip : float
    wheel_base : float
    steer_lock : float
    aero_cx : float
    aero_front_area : float
    rim_diameter_in : float
    tire_width_mm : float
    tire_height_ratio : float

    front_clift:float
    rear_clift:float

    gear_ratios:dict[int, float]
    rear_diff_ratio:float
    tickover_rpm:float
    torque_curve:dict[int, tuple[float, float]]


    @cached_property
    def axle_weight_ratio_front(self) -> float:
        """
        Computes the ratio of the distance from the center of gravity to the front axle
        over the total wheelbase.
        """
        return self.cg_to_rear_axle / self.wheel_base

    @cached_property
    def axle_weight_ratio_rear(self) -> float:
        """
        Computes the ratio of the distance from the center of gravity to the rear axle
        over the total wheelbase.
        """
        return self.cg_to_front_axle / self.wheel_base

    @cached_property
    def wheel_radius(self) -> float:
        """
        Computes the wheel radius in meters.
        """
        tire_height_mm = self.tire_width_mm * self.tire_height_ratio
        return (self.rim_diameter_in * 25.4 + 2 * tire_height_mm) / 2000

    @cached_property
    def aero_drag_coefficient(self) -> float:
        """
        Computes the aerodynamic drag coefficient (CdA = Cx * front_area).
        """
        return self.aero_cx * self.aero_front_area
    
    @cached_property
    def resist_air(self) -> float:
        """
        Computes the air resistance factor.
        """
        air_density = 1.225 # kg/m^3
        return 0.5 * air_density * self.aero_drag_coefficient
    

    @cached_property
    def lift_constant(self):
        """
        Computes the lift constant based on front and rear lift coefficients.
        """
        # Simplified average lift coefficient for the entire car
        total_lift = (self.front_clift + self.rear_clift) 
        air_density = 1.225 # kg/m^3
        return 0.5 * air_density * total_lift * self.aero_front_area
    

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

        self.car_physics = CarPhysicsProperties(
            mass=self._get_car_value('Car', 'mass'),
            cg_height=self._get_car_value('Car', 'GC height'),
            cg_to_front_axle=self._get_car_value('Front Axle', 'xpos'),
            cg_to_rear_axle=abs(self._get_car_value('Rear Axle', 'xpos')),
            tire_grip=self._get_car_value('Front Right Wheel', 'mu'),
            wheel_base=self._get_car_value('Front Axle', 'xpos') + abs(self._get_car_value('Rear Axle', 'xpos')),
            steer_lock=self._get_car_value('Steer', 'steer lock', default=21),
            aero_cx=self._get_car_value('Aerodynamics', 'Cx'),
            aero_front_area=self._get_car_value('Aerodynamics', 'front area'),
            rim_diameter_in=self._get_car_value('Rear Right Wheel', 'rim diameter'),
            tire_width_mm=self._get_car_value('Rear Right Wheel', 'tire width'),
            tire_height_ratio=self._get_car_value('Rear Right Wheel', 'tire height-width ratio'),
            front_clift=self._get_car_value('Aerodynamics', 'front Clift'),
            rear_clift=self._get_car_value('Aerodynamics', 'rear Clift'),

            gear_ratios=self.get_gear_ratios(),
            rear_diff_ratio=self._get_car_value('Rear Differential', 'ratio'),
            tickover_rpm=self._get_car_value('Engine', 'tickover', default=900),
            torque_curve=self.get_torque_curve()
        )

        # Physics constants
        self.physics_constants = {}
        self.extract_constants()

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

            linear_torque_data[rpm_from] = (m, b)

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
        self.physics_constants['PHYSICS_MASS'] = self._get_car_value('Car', 'mass')
        self.physics_constants['PHYSICS_CG_HEIGHT'] = self._get_car_value('Car', 'GC height')
        self.physics_constants['PHYSICS_CG_TO_FRONT_AXLE'] = self._get_car_value('Front Axle', 'xpos')
        self.physics_constants['PHYSICS_CG_TO_REAR_AXLE'] = abs(self._get_car_value('Rear Axle', 'xpos'))
        self.physics_constants['PHYSICS_TIRE_GRIP_STATIC'] = self._get_car_value('Front Right Wheel', 'mu')

        # Constants from track XML
        self.physics_constants['CRR_ROAD'] = self._get_track_value("Surfaces/rroad", "rolling resistance")
        self.physics_constants['FRICTION_ROAD'] = self._get_track_value("Surfaces/rroad", "friction")

        # Calculated constants
        self.physics_constants['PHYSICS_GRAVITY'] = 9.81
        self.physics_constants['PHYSICS_WHEEL_BASE'] = self.physics_constants['PHYSICS_CG_TO_FRONT_AXLE'] + self.physics_constants['PHYSICS_CG_TO_REAR_AXLE']
        steer_lock_deg = self._get_car_value('Steer', 'steer lock', default=21)
        self.physics_constants['STEER_MAX'] = math.radians(steer_lock_deg)

        # Tire radius calculation
        rim_diameter_in = self._get_car_value('Rear Right Wheel', 'rim diameter')
        tire_width_mm = self._get_car_value('Rear Right Wheel', 'tire width')
        tire_height_ratio = self._get_car_value('Rear Right Wheel', 'tire height-width ratio')
        tire_height_mm = tire_width_mm * tire_height_ratio
        tire_diameter_mm = rim_diameter_in * 25.4 + 2 * tire_height_mm
        self.physics_constants['WHEEL_RADIUS'] = (tire_diameter_mm / 2) / 1000.0

        # Aerodynamic constant calculation
        air_density = 1.225
        cx = self._get_car_value('Aerodynamics', 'Cx')
        front_area = self._get_car_value('Aerodynamics', 'front area')
        self.physics_constants['PHYSICS_AIR_RESIST'] = 0.5 * air_density * cx * front_area

        # Lift constant calculation
        front_clift = self._get_car_value('Aerodynamics', 'front Clift')
        rear_clift = self._get_car_value('Aerodynamics', 'rear Clift')
        self.physics_constants['PHYSICS_LIFT_CONSTANT'] = 0.5 * air_density * front_area * (front_clift + rear_clift)

        # Get torque curve and gear ratios
        self.torque_curve = self.get_torque_curve()
        self.gear_ratios = self.get_gear_ratios()

    def calculate_forces(self, throttle=0.0, brake=0.0):
        """
        Calculates all forces acting on the car for the current time step.
        :param throttle: Throttle input (0.0 to 1.0)
        :param brake: Brake input (0.0 to 1.0)
        :return: Total longitudinal force in Newtons.
        """
        v = self.velocity
        
        # 1. Drive force
        # Simplified automatic gear shifting logic for demonstration
        if self.gear == '1' and v > 15:
            self.gear = '2'
        elif self.gear == '2' and v > 30:
            self.gear = '3'
        elif self.gear == '3' and v > 50:
            self.gear = '4'
        elif self.gear == '4' and v > 80:
            self.gear = '5'
        elif self.gear == '5' and v > 120:
            self.gear = '6'

        gear_ratio = self.gear_ratios.get(self.gear, 0.0)
        diff_ratio = self._get_car_value('Rear Differential', 'ratio')
        
        # Calculate RPM from velocity
        wheel_rotation_rate = v / self.physics_constants['WHEEL_RADIUS']
        rpm = wheel_rotation_rate * gear_ratio * diff_ratio * 60 / (2 * math.pi)

        # Clamp RPM to tickover
        tickover_rpm = self._get_car_value('Engine', 'tickover', default=900)
        rpm = max(rpm, tickover_rpm)
        
        # Interpolate torque from the torque curve
        rpms = sorted(self.torque_curve.keys())
        if not rpms:
            torque = 0.0
        elif rpm <= rpms[0]:
            torque = self.torque_curve[rpms[0]]
        elif rpm >= rpms[-1]:
            torque = self.torque_curve[rpms[-1]]
        else:
            # Use bisect for efficient lookup
            idx = bisect.bisect_left(rpms, rpm)
            rpm1, rpm2 = rpms[idx - 1], rpms[idx]
            tq1, tq2 = self.torque_curve[rpm1], self.torque_curve[rpm2]
            torque = tq1 + (tq2 - tq1) * (rpm - rpm1) / (rpm2 - rpm1)
        
        drive_force = torque * gear_ratio * diff_ratio / self.physics_constants['WHEEL_RADIUS']
        
        # 2. Air drag force
        air_drag_force = -self.physics_constants['PHYSICS_AIR_RESIST'] * v * abs(v)
        
        # 3. Rolling resistance force (now a constant force)
        normal_force_static = self.physics_constants['PHYSICS_MASS'] * self.physics_constants['PHYSICS_GRAVITY']
        rolling_resistance_force = -self.physics_constants['CRR_ROAD'] * normal_force_static

        # 4. Total force on the car
        total_force = (drive_force * throttle) + (self._get_car_value('Brake System', 'max pressure', default=26300.0) * brake) + air_drag_force + rolling_resistance_force

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
        acceleration = total_force / self.physics_constants['PHYSICS_MASS']
        self.velocity += acceleration * self.dt
        self.position += self.velocity * self.dt

    def run_simulation(self, steps, throttle_input, brake_input):
        """
        Runs a simplified simulation for a number of steps.
        :param steps: Number of simulation steps.
        :param throttle_input: A list of throttle values for each step.
        :param brake_input: A list of brake values for each step.
        """
        for i in range(steps):
            if i < len(throttle_input):
                throttle = throttle_input[i]
            else:
                throttle = 0.0
            
            if i < len(brake_input):
                brake = brake_input[i]
            else:
                brake = 0.0
            
            self.update(throttle, brake)
            print(f"Time Step: {i*self.dt:.2f}s | Velocity: {self.velocity:.2f} m/s | Gear: {self.gear}")

# To use this class, you would need to provide the XML files.
# Example of a simplified XML content for testing purposes:

# Initialize the engine
engine = PhysicsEngine("input/cars/car1-trb1.xml")
print("Engine initialized with the following constants:")
for key, value in engine.physics_constants.items():
    print(f"- {key}: {value}")

# Define a simple throttle input pattern
throttle_pattern = [1.0] * 50 + [0.5] * 50 + [0.0] * 50
brake_pattern = [0.0] * 100 + [1.0] * 50

# Run the simulation
print("\nRunning simulation...")
engine.run_simulation(150, throttle_pattern, brake_pattern)