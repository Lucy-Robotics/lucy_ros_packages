"""ROS 2 node: poll SHM dirty bits and forward Modbus RTU writes to a board."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    serial = None
    list_ports = None

from lucy_modbus_bridge.shm import (
    REGISTER_COUNT,
    build_write_single,
    get_dirty,
    open_board_shm,
    post_sem,
    read_register,
    set_clean,
    wait_sem,
)


class ModbusBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('lucy_modbus_bridge')
        self.declare_parameter('node_name', 'lucy_hardware_interface')
        self.declare_parameter('serial_id', '')
        self.declare_parameter('slave_address', 1)
        self.declare_parameter('baud', 115200)
        self.declare_parameter('poll_hz', 50.0)
        # 0 = do not filter by USB VID/PID (match serial_id / any ACM).
        # Legacy Custom Servo2040 firmware used 0x16C0/0x27DD; Pico CDC is 0x2E8A.
        self.declare_parameter('vid', 0)
        self.declare_parameter('pid', 0)
        self.declare_parameter('shm_wait_sec', 60.0)

        self._node_name = self.get_parameter('node_name').get_parameter_value().string_value
        self._serial_id = self.get_parameter('serial_id').get_parameter_value().string_value.strip()
        self._slave = int(self.get_parameter('slave_address').value)
        self._baud = int(self.get_parameter('baud').value)
        self._vid = int(self.get_parameter('vid').value)
        self._pid = int(self.get_parameter('pid').value)
        shm_wait = float(self.get_parameter('shm_wait_sec').value)
        poll_hz = float(self.get_parameter('poll_hz').value)

        if serial is None:
            raise RuntimeError('pyserial is required for lucy_modbus_bridge')

        from lucy_modbus_bridge.shm import shm_object_names

        reg_name, _, _ = shm_object_names(self._node_name)
        self.get_logger().info(
            f'waiting up to {shm_wait:.0f}s for HI SHM {reg_name} '
            f'(node_name={self._node_name})'
        )
        self._shm = open_board_shm(self._node_name, timeout_sec=shm_wait)
        self._port = None
        self._warn_missing_serial = True
        self._try_open_serial()
        period = 1.0 / max(poll_hz, 1.0)
        self._timer = self.create_timer(period, self._on_timer)
        if self._port is not None:
            self.get_logger().info(
                f'bridge ready: node_name={self._node_name} '
                f'shm={self._shm.shm_node_name} '
                f'serial={self._port.port} slave={self._slave}'
            )
        else:
            self.get_logger().warn(
                f'bridge idle (no USB yet): node_name={self._node_name} '
                f'serial_id={self._serial_id!r} — will retry each poll'
            )

    def _try_open_serial(self) -> bool:
        """Open USB CDC when present; return False without raising if absent."""
        if self._port is not None:
            return True
        port = self._find_serial()
        if port is None:
            if self._warn_missing_serial:
                vid_s = f'{self._vid:#x}' if self._vid else 'any'
                pid_s = f'{self._pid:#x}' if self._pid else 'any'
                self.get_logger().warn(
                    f'no USB serial matching vid={vid_s} pid={pid_s} '
                    f'serial_id={self._serial_id!r}; bridge will retry'
                )
                self._warn_missing_serial = False
            return False
        self._port = port
        self._warn_missing_serial = True
        self.get_logger().info(f'USB serial open: {self._port.port}')
        return True

    def _find_serial(self):
        needle = self._serial_id.lower()
        candidates = []
        for info in list_ports.comports():
            if self._vid and info.vid != self._vid:
                continue
            if self._pid and info.pid != self._pid:
                continue
            hay = ' '.join(
                filter(None, [info.device, info.serial_number or '', info.hwid or ''])
            ).lower()
            if needle and needle not in hay:
                continue
            candidates.append(info)
        if len(candidates) == 1:
            info = candidates[0]
            return serial.Serial(info.device, self._baud, timeout=0.05)
        if len(candidates) > 1 and needle:
            # Prefer the port whose serial_number equals the flash id.
            for info in candidates:
                if (info.serial_number or '').lower() == needle:
                    return serial.Serial(info.device, self._baud, timeout=0.05)
            info = candidates[0]
            return serial.Serial(info.device, self._baud, timeout=0.05)
        return None

    def _on_timer(self) -> None:
        if not self._try_open_serial():
            return
        # Serialize against LucySystemHardware::write via the shared semaphore.
        try:
            wait_sem(self._shm)
        except OSError as exc:
            self.get_logger().error(f'sem_wait failed: {exc}')
            return
        try:
            dirty_regs: list[int] = []
            for reg in range(REGISTER_COUNT):
                if get_dirty(self._shm.header_mm, reg):
                    dirty_regs.append(reg)
            if not dirty_regs:
                return

            for reg in dirty_regs:
                value = read_register(self._shm.reg_mm, reg)
                frame = build_write_single(self._slave, reg, value)
                try:
                    self._port.write(frame)
                    # Drain response (echo for FC06) without blocking long.
                    time.sleep(0.002)
                    _ = self._port.read(16)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(f'Modbus write failed reg={reg}: {exc}')
                    try:
                        self._port.close()
                    except Exception:  # noqa: BLE001
                        pass
                    self._port = None
                    self._warn_missing_serial = True
                    continue
                set_clean(self._shm.header_mm, reg)
        finally:
            try:
                post_sem(self._shm)
            except OSError as exc:
                self.get_logger().error(f'sem_post failed: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ModbusBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
