## Import libary
import pychrono.core as chr
import pychrono.irrlicht as chrirr
import numpy as np
import matplotlib.pyplot as plt
import MechanismSystem as mech

# Add absolute path chrono engine data in environment
#chr.SetChronoDataPath('D:/Programms/anaconda3/pkgs/pychrono-7.0.0-py39_0/Library/data/')

# ## Initilize chrono system
system = chr.ChSystemNSC()
system.Set_G_acc(chr.ChVectorD(0,0,0)) # Set zero gravity 

# Parameters
len_link_2 = 0.6 #[m]
len_BC = 0.2 #[m]

# ## Simulate
mechanism = mech.MechanismSystem(system,len_link_2,len_BC)
mechanism.CreateBodies()
mechanism.AddJoints()
arr_time, ang, omg = mechanism.Simulate()

# Plot graphics
plt.figure(figsize=(10,6))

plt.subplot(2,1,1)
plt.title("Motor Position")
plt.plot(arr_time,ang)
plt.grid(True)
plt.ylabel("teta, [rad]")
plt.subplot(2,1,2)
plt.title("Motor Velocity")
plt.plot(arr_time,omg)
plt.grid(True)
plt.xlabel("Time, [s]")
plt.ylabel("omega, [rad/s]")
plt.show()