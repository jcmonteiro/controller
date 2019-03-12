import numpy as np
import unittest

import sys
sys.path.append('../build')
from pid_control import PID, PIDInput, PIDSettings, PIDParameters
from linear_system import LinearSystem, IntegrationMethod

class ProgressBar:
    def __init__(self, maxval = 100, width = 20):
        self.max = float(maxval)
        self.width = int(width)

    def update(self, value):
        if (value < 0):
            value = 0
        elif (value > self.max):
            value = self.max

        rate = value / self.max
        fmt = "[%-" + str(self.width) + "s] %d%%"

        # reset the cursor to the beginning of the line
        sys.stdout.write('\r')
        # print the progress bar
        sys.stdout.write(fmt % ('=' * int(rate * self.width), 100 * rate))
        # force everything to be printed
        sys.stdout.flush()

def computeError(vec1, vec2):
    return np.amax( np.absolute( vec1 - vec2 ) )

class TestPIDControl(unittest.TestCase):
    def test_pid(self):
        pid_settings = PIDSettings()
        pid_settings.B = 1
        pid_settings.Ki = 5
        pid_settings.Kd = 0.2
        pid_settings.Kp = 19
        pid_settings.Tt = 1 / pid_settings.Ki
        pid_settings.Ts = 1
        pid_settings.YMin = -10
        pid_settings.YMax = 10

        self.do_test(PID(), pid_settings, PIDInput, PIDParameters)

    def do_test(self, controller, settings, Input, Parameters):
        params = Parameters()
        params.setSettings( [settings] )
        params.integration_method = IntegrationMethod.TUSTIN
        params.wrap = False
        params.saturate = True
        controller.configParameters(params)
        ctrl_in = Input.create(1.0, 0.9, 0.2, 0.3, 0)
        n_turns = 10
        time = 0
        dt = 1
        history = [None] * n_turns
        controller.restart()
        for k in range(0, n_turns):
            time += dt
            ctrl_in.time = LinearSystem.getTimeFromSeconds(time)
            out = controller.update(ctrl_in)
            history[k] = out.getValue()[0]
        controller.restart()
        time = 0
        for k in range(0, n_turns):
            time += dt
            ctrl_in.time = LinearSystem.getTimeFromSeconds(time)
            out = controller.update(ctrl_in)
            self.assertTrue(out.getValue() == history[k])

if __name__ == "__main__":
    unittest.main()
