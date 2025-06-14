import math
from dataclasses import dataclass

@dataclass
class AckermanResult:
    inner_wheel_angle_deg: float
    outer_wheel_angle_deg: float
    speed_ratio: float


class AckermanCalculator:
    @staticmethod
    def calculate_ackerman(w: float, x: float) -> AckermanResult:
        """
        Calculates the Ackerman angles and speed ratio based on the track width and turning radius.

        Args:
            w (float): Track width (meters)
            x (float): Distance to turn center from vehicle center (meters), positive = right turn

        Returns:
            AckermanResult: inner angle (deg), outer angle (deg), speed ratio
        """
        if w <= 0:
            raise ValueError("Track width must be positive")
        
        if x == 0:
            return AckermanResult(0.0, 0.0, 1.0)  # Going straight

        # For a right turn (x > 0), the left wheel is the inner wheel
        # For a left turn (x < 0), the right wheel is the inner wheel
        is_right_turn = x > 0
        inner_to_center = abs(x) - w / 2
        outer_to_center = abs(x) + w / 2

        inner_angle_rad = math.atan(w / (2 * inner_to_center))
        outer_angle_rad = math.atan(w / (2 * outer_to_center))

        inner_angle_deg = math.degrees(inner_angle_rad)
        outer_angle_deg = math.degrees(outer_angle_rad)

        speed_ratio = outer_to_center / inner_to_center

        return AckermanResult(inner_angle_deg, outer_angle_deg, speed_ratio)

    @staticmethod
    def calculate_outer_from_inner(track_width_mm: float, inner_angle_deg: float) -> AckermanResult:
        """
        Calculates the outer wheel angle and speed ratio given the inner wheel angle.

        Args:
            track_width_mm (float): Track width in mm
            inner_angle_deg (float): Inner wheel angle in degrees

        Returns:
            AckermanResult: outer angle (deg), speed ratio
        """
        if track_width_mm <= 0:
            raise ValueError("Track width must be positive")

        if abs(inner_angle_deg) > 90:
            raise ValueError("Wheel angle cannot exceed ±90 degrees")

        if inner_angle_deg == 0:
            return AckermanResult(0.0, 0.0, 1.0)

        sign = math.copysign(1, inner_angle_deg)
        inner_angle_rad = math.radians(abs(inner_angle_deg))

        inner_radius = track_width_mm / (2 * math.tan(inner_angle_rad))  # in mm
        center_dist = inner_radius + track_width_mm / 2
        outer_radius = center_dist + track_width_mm / 2

        outer_angle_rad = math.atan(track_width_mm / (2 * outer_radius))
        outer_angle_deg = sign * math.degrees(outer_angle_rad)

        speed_ratio = outer_radius / inner_radius

        return AckermanResult(0.0, outer_angle_deg, speed_ratio)
