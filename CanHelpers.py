import collections
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Any
import can


# CAN helpers
def get_message_data(can_frame: Any) -> bytes | bytearray | None:
    if can_frame:
        data = getattr(can_frame, "data", None)
        if isinstance(data, (bytes, bytearray)):
            return data
    return None


def crc(can_id: int, payload: list[int]) -> int:
    return (can_id + sum(payload)) & 0xFF

def build(can_id: int, payload: list[int]) -> list:
    return payload + [crc(can_id, payload)]


@dataclass
class Frame:
    can_id: int
    code: int           # data[0]
    data: bytes
    timestamp: float = field(default_factory=time.monotonic)

    @classmethod
    def from_can_message(cls, msg: can.Message) -> "Frame":
        return cls(
            can_id=msg.arbitration_id,
            code=msg.data[0],
            data=bytes(msg.data),
        )


class CanBus:
    """
    Owns the physical CAN interface and the single listener thread.
    MotorInterfaces register handlers by CAN ID; the listener dispatches
    each received frame to the matching handler on a dedicated callback
    thread so handlers never block the listener.
    """

    def __init__(self, bus_type, usb_port, bitrate):
        self._bus = can.Bus(interface=bus_type, channel=usb_port, bitrate=bitrate)
        # id -> list of handler callables
        self._handlers: dict[int, list[Callable[[Frame], None]]] = {}
        self._lock = threading.Lock()

        # Callback executor: one thread with a queue so handlers
        # don't block each other or the listener
        self._cb_queue: collections.deque = collections.deque()
        self._cb_event = threading.Event()
        self._cb_thread = threading.Thread(
            target=self._callback_loop, daemon=True, name="can-callbacks"
        )
        self._cb_thread.start()

        self._listener = threading.Thread(
            target=self._listen_loop, daemon=True, name="can-listener"
        )
        self._listener.start()

    # ── Registration ──────────────────────────────────────────────────────────
    def register(self, can_id: int, handler: Callable[[Frame], None]):
        with self._lock:
            self._handlers.setdefault(can_id, []).append(handler)

    def unregister(self, can_id: int, handler: Callable[[Frame], None]):
        with self._lock:
            if can_id in self._handlers:
                self._handlers[can_id].remove(handler)

    # ── Send ──────────────────────────────────────────────────────────────────
    def send(self, can_id: int, data: list):
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        self._bus.send(msg)

    # ── Internal threads ──────────────────────────────────────────────────────
    def _listen_loop(self):
        while True:
            try:
                msg = self._bus.recv(timeout=0.02)
                if msg is None:
                    continue
                frame = Frame.from_can_message(msg)
                if isinstance(msg.data, (bytes, bytearray)):
                    bytes_str = " ".join(f"{b:02x}" for b in msg.data)
                    if msg.data[0] not in [0x31, 0x82, 0x01, 0xF3, 0x3a, 0x91, 0x8C]:
                        print("%02X: %s" % (msg.arbitration_id, bytes_str))
                with self._lock:
                    handlers = list(self._handlers.get(frame.can_id, []))
                if handlers:
                    self._cb_queue.append((handlers, frame))
                    self._cb_event.set()
            except Exception as e:
                print(f"[CanBus] listener error: {e}")

    def _callback_loop(self):
        while True:
            self._cb_event.wait()
            self._cb_event.clear()
            while self._cb_queue:
                handlers, frame = self._cb_queue.popleft()
                for handler in handlers:
                    try:
                        handler(frame)
                    except Exception as e:
                        print(f"[CanBus] callback error: {e}")

    def close(self):
        self._bus.stop_all_periodic_tasks()
        self._bus.shutdown()
