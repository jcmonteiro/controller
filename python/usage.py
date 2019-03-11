from pid_control import PID, PIDSettings, PIDParameters, PIDInput
from linear_system import LinearSystem, IntegrationMethod

# settings for the first degree-of-freedom
s1 = PIDSettings()
s1.Kp = 4
s1.Ki = 20
s1.Kd = 2
s1.Ts = 0.002
s1.Tt = 1 / s1.Ki
s1.YMin = 0
s1.YMax = 10

# settings for the second degree-of-freedom
s2 = PIDSettings()
s2.Kp = 5
s2.Ki = 16
s2.Kd = 3
s2.Ts = 0.002 # they all must be the same!!!
s2.Tt = 1 / s1.Ki
s2.YMin = 0
s2.YMax = 10

# PID parameters using each individual setting
params = PIDParameters()
params.setSettings( [s1, s2] )
params.wrap = False
params.saturate = True
params.integration_method = IntegrationMethod.TUSTIN

# configure PID instance
controller = PID()
controller.configParameters( params )

# ... code is running ...
# ... inside a mainloop to update the controller ...
ctrl_in = PIDInput()
ctrl_in.time = LinearSystem.getTimeFromSeconds(0.2)
ctrl_in.ref = [0.1, 0.2]
ctrl_in.sig = [0.3, 0.4]
ctrl_in.dref = [0, 0]
ctrl_in.dsig = [0, 0]
ctrl_in.sat = [0, 0] # last PID output after saturation

out = controller.update( ctrl_in )
print(out.getValue())