import math
import struct


def counts_to_degrees(counts: int, counts_per_revolution: int) -> float:
    return counts / counts_per_revolution * 360.0

def counts_to_position_bytes(counts: int) -> bytes:
    position_bytes = struct.pack('>i', counts)[1:]  # signed 24-bit big-endian
    return position_bytes

def degrees_to_counts(degrees: float, counts_per_revolution: int) -> int:
    return round(degrees / 360.0 * counts_per_revolution)

def degrees_to_position_bytes(degrees: float, counts_per_revolution: int) -> bytes:
    counts = degrees_to_counts(degrees, counts_per_revolution)
    position_bytes = counts_to_position_bytes(counts)
    return position_bytes

def rpm_to_speed_bytes(speed_rpm: int) -> list[int]:
    s_hi = (speed_rpm >> 8) & 0x0F
    s_lo = speed_rpm & 0xFF
    return [s_hi, s_lo]

# Conversions
def angular_acceleration_to_byte(angular_acceleration: float) -> int:
    acceleration_mks = math.floor(256.0 - (2000.0 * math.pi / (3.0 * angular_acceleration)))
    if acceleration_mks < 1:
        acceleration_mks = 1
    return acceleration_mks

def linear_acceleration_to_byte(linear_acceleration:float, mm_per_revolution: float) -> int:
    acceleration_mks = math.floor(256.0 - ((1000.0 * mm_per_revolution) / (3.0 * linear_acceleration)))
    if acceleration_mks < 1:
        acceleration_mks = 1
    return acceleration_mks

def mks_acceleration_to_angular_acceleration(acceleration_mks: int) -> float:
    angular_acceleration = 2000.0 * math.pi / (3.0 * (256.0 - acceleration_mks))
    return angular_acceleration

def mks_acceleration_to_linear_acceleration(acceleration_mks: int, mm_per_revolution: float) -> float:
    linear_acceleration = 1000.0 * mm_per_revolution / (3.0 * (256.0 - acceleration_mks))
    return linear_acceleration

def linear_velocity_to_rpm(linear_velocity: float, mm_per_revolution: float) -> int:
    speed_rpm = int(math.floor(60.0 * linear_velocity / mm_per_revolution))
    return speed_rpm

def rpm_to_linear_velocity(speed_rpm: float, mm_per_revolution: float) -> float:
    linear_velocity = speed_rpm * mm_per_revolution / 60.0
    return linear_velocity_to_rpm(linear_velocity, mm_per_revolution)

def linear_velocity_to_speed_bytes(linear_velocity: float, mm_per_revolution:float) -> list[int]:
    speed_rpm = linear_velocity_to_rpm(linear_velocity, mm_per_revolution)
    speed_bytes = rpm_to_speed_bytes(speed_rpm)
    return speed_bytes

def position_degrees_to_linear_position(position_degrees: float, mm_per_revolution: float) -> float:
    linear_position = mm_per_revolution * position_degrees / 360.0
    return linear_position

def linear_position_to_position_degrees(linear_position: float, mm_per_revolution: float) -> float:
    position_degrees = 360.0 * linear_position / mm_per_revolution
    return position_degrees

