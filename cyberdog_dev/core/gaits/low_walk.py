"""Conservative low-walk adapter for passing under limit poles."""

import time


def execute_low_walk(dog, duration_sec: float, speed: float = 0.12) -> bool:
    dog.set_velocity(speed, 0.0, 0.0, body_height=0.16, step_height=(0.03, 0.03))
    if duration_sec > 0.0:
        time.sleep(duration_sec)
    dog.stop()
    return True
