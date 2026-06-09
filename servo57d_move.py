"""
MKS SERVO57D CAN - Enable, move to 90°, back to 0°, disable
CAN ID: 12 (0x0C)

Hardware: CANable (slcan or socketcan interface)
Protocol: MKS CAN V1.0.9 manual

CRC: CHECKSUM 8-bit = (CAN_ID + all_payload_bytes) & 0xFF
Encoder: 16384 counts/revolution  →  90° = 4096 (0x1000)
"""

import serial
import can
import time
import struct

# ── Configuration ────────────────────────────────────────────────────────────
USB_PORT     = "COM10"
CAN_BITRATE = 500000
CAN_ID       = 0x0C          # Device CAN ID (12 decimal)
BUS_TYPE      = "slcan"       # "slcan" for CANable default firmware, "socketcan" for gs_usb
SPEED_RPM    = 600           # Movement speed  (0–3000 RPM)
ACCEL        = 150           # Acceleration    (0–255)
# ─────────────────────────────────────────────────────────────────────────────

# 16384 encoder counts per full revolution
COUNTS_PER_REV = 16384
def degrees_to_counts(deg: float) -> int:
    return round(deg / 360.0 * COUNTS_PER_REV)

def crc(can_id: int, payload: list[int]) -> int:
    """8-bit checksum: (CAN_ID + all payload bytes) & 0xFF"""
    return (can_id + sum(payload)) & 0xFF

def build_frame(can_id: int, payload: list[int]) -> list[int]:
    """Append CRC to payload and return complete data bytes."""
    return payload + [crc(can_id, payload)]

def send_and_recv(bus: can.BusABC, can_id: int, data: list[int],
                  timeout: float = 2.0) -> can.Message | None:
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    bus.send(msg)
    print(f"  TX  id=0x{can_id:03X}  data={[f'0x{b:02X}' for b in data]}")
    # Wait for reply (same CAN ID echoed back from the driver)
    deadline = time.time() + timeout
    while time.time() < deadline:
        reply = bus.recv(timeout=0.1)
        if reply and reply.arbitration_id == can_id:
            print(f"  RX  id=0x{reply.arbitration_id:03X}  data={[f'0x{b:02X}' for b in reply.data]}")
            return reply
    print("  RX  (no reply within timeout)")
    return None

# ── Command builders ──────────────────────────────────────────────────────────

def cmd_enable(can_id: int, enable: bool) -> list[int]:
    """F3H – Set motor enable state.  DLC=3"""
    payload = [0xF3, 0x01 if enable else 0x00]
    return build_frame(can_id, payload)

def cmd_set_work_mode(can_id: int, mode: int) -> list[int]:
    """0x82 – Set work mode.  DLC=3
    0x05 = SR_vFOC (Bus Interface FOC Mode)  ← recommended
    0x04 = SR_CLOSE (Bus closed-loop)
    0x03 = SR_OPEN  (Bus open-loop)
    """
    payload = [0x82, mode]
    return build_frame(can_id, payload)

def cmd_abs_move(can_id: int, degrees: float,
                 speed_rpm: int = SPEED_RPM, accel: int = ACCEL) -> list[int]:
    """F5H – Absolute coordinate move.  DLC=8
    Speed encoding: high nibble of byte2 = speed[11:8], byte3 = speed[7:0]
    absAxis: int24, big-endian (bytes 5-7 in manual = our payload bytes 4-6)
    """
    abs_counts = degrees_to_counts(degrees)

    # Speed is 12-bit split across two bytes: [speed_hi(4 bits) | speed_lo(8 bits)]
    speed_hi = (speed_rpm >> 8) & 0x0F
    speed_lo =  speed_rpm       & 0xFF

    # absAxis as signed 24-bit big-endian
    axis_bytes = struct.pack(">i", abs_counts)[1:]   # drop the MSB of int32 → 3 bytes

    payload = [0xF5, speed_hi, speed_lo, accel,
               axis_bytes[0], axis_bytes[1], axis_bytes[2]]
    return build_frame(can_id, payload)

# ── Main sequence ─────────────────────────────────────────────────────────────

def main():
    print(f"Connecting to CAN bus ({BUS_TYPE} on {USB_PORT}) …")
    with can.Bus(interface=BUS_TYPE, channel="COM10", bitrate=CAN_BITRATE) as bus:

        # 1. Enable motor
        print("\n[1] Enabling motor …")
        data = cmd_enable(CAN_ID, enable=True)
        reply = send_and_recv(bus, CAN_ID, data)
        if reply and reply.data[1] != 0x01:
            print("  WARNING: enable command reported failure")
        time.sleep(0.2)

        # 2. Move to 90°
        print(f"\n[2] Moving to 90° ({degrees_to_counts(90)} counts) …")
        data = cmd_abs_move(CAN_ID, degrees=18000.0)
        reply = send_and_recv(bus, CAN_ID, data, timeout=5.0)
        # status=1 → starting, status=2 → complete
        if reply:
            status = reply.data[1]
            print(f"  Move status: {status} ({'starting' if status==1 else 'complete' if status==2 else 'other'})")

        # Poll until complete (status == 2) or timeout
        deadline = time.time() + 10.0
        while time.time() < deadline:
            reply = bus.recv(timeout=0.5)
            if reply and reply.arbitration_id == CAN_ID and reply.data[0] == 0xF5:
                if reply.data[1] == 2:
                    print("  Motor reached 90° ✓")
                    break

        # 3. Wait 1 second
        print("\n[3] Waiting 1 second …")
        time.sleep(1.0)

        # 4. Move back to 0°
        print(f"\n[4] Moving back to 0° …")
        data = cmd_abs_move(CAN_ID, degrees=0.0)
        reply = send_and_recv(bus, CAN_ID, data, timeout=5.0)
        if reply:
            status = reply.data[1]
            print(f"  Move status: {status}")

        deadline = time.time() + 10.0
        while time.time() < deadline:
            reply = bus.recv(timeout=0.5)
            if reply and reply.arbitration_id == CAN_ID and reply.data[0] == 0xF5:
                if reply.data[1] == 2:
                    print("  Motor reached 0° ✓")
                    break

        # 5. Disable motor
        print("\n[5] Disabling motor …")
        data = cmd_enable(CAN_ID, enable=False)
        send_and_recv(bus, CAN_ID, data)

    print("\nDone.")


if __name__ == "__main__":
    main()
