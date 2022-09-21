### Import libaries and etc.
import pychrono.core as chrono # Main libary for chrono simulation
import pychrono.irrlicht as chrirr # Vizualization libary
import numpy as np # Arrays calculation
import matplotlib.pyplot as plt # For plot graphics
import AcrobotSystem as acr # Class with acrobot's model

### Parameters
len_rod_1 = 0.3 # [m] length first rod
len_rod_2 = 0.6 # [m] length second rod
density = 1000 # [kg/m^3] Density

# Add absolute path chrono engine data in environment
#chrono.SetChronoDataPath('D:/Programms/anaconda3/pkgs/pychrono-7.0.0-py39_0/Library/data/')

## Init chrono system
system = chrono.ChSystemNSC() # Create var non-smooth chrono system

# Create body material for initilize collision model bodies
body_material = chrono.ChMaterialSurfaceNSC()
body_material.SetFriction(0.6)
body_material.SetDampingF(0.4)


### Simulate

# Object of acrobot
acrobot = acr.AcrobotSystem(system,len_rod_1,len_rod_2, body_material)

# Initilize joint
acrobot.AddJoints()

# Simulate system (time_simulation, time_step)
data_result = acrobot.Simulate(5,0.01)

### Plot graphics
plt.figure(figsize=(8,6))

plt.subplot(2,1,1)
plt.title("Joint Angles")
plt.plot(data_result['time'],data_result['angle_1'],data_result['time'],data_result['angle_2'])
plt.legend(["q1", "q2"])
plt.grid(True)
plt.ylabel("q1, q2, [rad]")
plt.subplot(2,1,2)
plt.title("Angular Velocity")
plt.plot(data_result['time'],data_result['omega_1'],data_result['time'],data_result['omega_2'])
plt.legend(["omega1","omega2"])
plt.grid(True)
plt.xlabel("Time, [s]")
plt.ylabel("dq1, dq2, [rad/s]")
plt.show()