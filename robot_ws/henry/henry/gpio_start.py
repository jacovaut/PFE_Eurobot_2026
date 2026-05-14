import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


class GpioInput:
    def __init__(self, pin: int, active_high: bool = True):
        self.pin = pin
        self.active_high = active_high
        self._backend = None
        self._device = None

        self._init_gpiozero()
        if self._backend is None:
            self._init_rpi_gpio()
        if self._backend is None:
            self._init_sysfs()

        if self._backend is None:
            raise RuntimeError(
                'No GPIO backend available. Install python3-gpiozero or '
                'python3-rpi.gpio, or enable /sys/class/gpio access.'
            )

    def is_active(self) -> bool:
        if self._backend == 'gpiozero':
            value = bool(self._device.value)
        elif self._backend == 'rpi_gpio':
            value = bool(self._device.input(self.pin))
        else:
            value_path = os.path.join(self._device, 'value')
            with open(value_path, 'r', encoding='utf-8') as handle:
                value = handle.read().strip() == '1'

        return value if self.active_high else not value

    def close(self) -> None:
        if self._backend == 'gpiozero':
            self._device.close()
        elif self._backend == 'rpi_gpio':
            self._device.cleanup(self.pin)

    def _init_gpiozero(self) -> None:
        try:
            from gpiozero import DigitalInputDevice
        except ImportError:
            return

        self._device = DigitalInputDevice(self.pin, pull_up=not self.active_high)
        self._backend = 'gpiozero'

    def _init_rpi_gpio(self) -> None:
        try:
            import RPi.GPIO as GPIO
        except ImportError:
            return

        GPIO.setmode(GPIO.BCM)
        pull = GPIO.PUD_DOWN if self.active_high else GPIO.PUD_UP
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=pull)
        self._device = GPIO
        self._backend = 'rpi_gpio'

    def _init_sysfs(self) -> None:
        gpio_dir = os.path.join('/sys/class/gpio', f'gpio{self.pin}')
        if not os.path.exists(gpio_dir):
            try:
                with open('/sys/class/gpio/export', 'w', encoding='utf-8') as handle:
                    handle.write(str(self.pin))
            except OSError:
                return

        direction_path = os.path.join(gpio_dir, 'direction')
        try:
            with open(direction_path, 'w', encoding='utf-8') as handle:
                handle.write('in')
        except OSError:
            return

        self._device = gpio_dir
        self._backend = 'sysfs'


def wait_for_gpio_start(
    pin: int,
    active_high: bool = True,
    poll_sec: float = 0.05,
    logger=None,
) -> None:
    gpio = GpioInput(pin, active_high)
    try:
        if logger is not None:
            level = 'high' if active_high else 'low'
            logger.info(f'Waiting for GPIO {pin} to go {level} before starting Henry')

        while not gpio.is_active():
            time.sleep(poll_sec)

        if logger is not None:
            logger.info(f'GPIO {pin} start signal detected; launching Henry')
    finally:
        gpio.close()


class GpioMatchStart(Node):
    def __init__(self):
        super().__init__('gpio_match_start')

        self.declare_parameter('pin', 22)
        self.declare_parameter('active_high', True)
        self.declare_parameter('poll_sec', 0.05)
        self.declare_parameter('start_topic', '/match/start')
        self.declare_parameter('latch_start', True)

        pin = int(self.get_parameter('pin').value)
        active_high = as_bool(self.get_parameter('active_high').value)
        poll_sec = float(self.get_parameter('poll_sec').value)
        start_topic = self.get_parameter('start_topic').value

        start_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._gpio = GpioInput(pin, active_high)
        self._pin = pin
        self._was_active = False
        self._started = False
        self._latch_start = as_bool(self.get_parameter('latch_start').value)
        self._pub = self.create_publisher(Bool, start_topic, start_qos)
        self.create_timer(max(poll_sec, 0.01), self._on_timer)

        level = 'high' if active_high else 'low'
        self.get_logger().info(
            f'Watching GPIO {pin}; publishing True to {start_topic} '
            f'when it goes {level}.'
        )

    def destroy_node(self):
        self._gpio.close()
        super().destroy_node()

    def _on_timer(self):
        active = self._gpio.is_active()
        if active and not self._was_active and not self._started:
            msg = Bool()
            msg.data = True
            self._pub.publish(msg)
            self._started = self._latch_start
            self.get_logger().info(
                f'GPIO {self._pin} start detected; match/start=True published.'
            )
        elif not active and not self._latch_start:
            msg = Bool()
            msg.data = False
            self._pub.publish(msg)

        self._was_active = active


def main(args=None):
    rclpy.init(args=args)
    node = GpioMatchStart()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
