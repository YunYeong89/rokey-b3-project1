"""lgpio stand-in for running the unchanged firmware (firmware/car_turn_lift.py) on a simulator."""


class SimGPIO:
    """lgpio stand-in: pin duty 0..100 (a written level is 0 or 100) and servo pulse widths (0 = off)."""
    error = OSError

    def __init__(self):
        self.pins, self.servo = {}, {}

    def gpiochip_open(self, chip):
        return 0

    def gpio_claim_output(self, h, pin, level=0):
        self.pins[pin] = 100. * level

    def gpio_write(self, h, pin, level):
        self.pins[pin] = 100. * level

    def tx_pwm(self, h, pin, freq, duty):
        self.pins[pin] = float(duty)

    def tx_servo(self, h, pin, width, freq=50):
        self.servo[pin] = width

    def gpiochip_close(self, h):
        pass

    def side(self, pins):
        """(duty -1..1, braked) of a motor channel: TB6612 EN / IN1 / IN2 (H H short brake, L L coast, PWM low = short
        brake) or a two-pin a / b driver."""
        level = lambda pin: self.pins.get(pin, 0.)
        if 'en' in pins:
            a, b = level(pins['in1']) > 50, level(pins['in2']) > 50
            return (0., a) if a == b else ((1 if a else -1) * level(pins['en']) / 100, True)
        a, b = level(pins['a']), level(pins['b'])
        return (0., a > 50) if a == b else ((a - b) / 100, True)

    def angle(self, pin):
        width = self.servo.get(pin, 0)
        return (width - 500) * 180 / 2000 if width else None
