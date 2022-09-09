import pychrono.core as chrono
import pychrono.irrlicht as chrirr
import numpy as np


#---------------------------------
# Function LQR controller 
# q - feedbck variables
#---------------------------------
def Feedback(q):
    #Controller's coefficients 
    K = np.asarray([-95.33425181, -44.15974824, -29.72448497, -17.59465727])
    q1 = q[0].GetMotorRot() 
    q2 = q[1].GetMotorRot()
    d_q1 = q[0].GetMotorRot_dt()
    d_q2 = q[1].GetMotorRot_dt()
    
    # state vector
    x = np.asarray([q1, q2, d_q1, d_q2]).T
    
    # сontrolling influence
    u = -K @ x # 
    
    #print(u)
    return u

class AcrobotSystem():
    # chrono_system - object chrono system
    # len_1 - length first rod
    # len_2 - length second rod
    # body_material - object chrono material
    def __init__(self, chrono_system, len_1, len_2, body_material):
        self.chrono_system = chrono_system
        
        self.len_rod_1 = len_1 # [m]
        self.len_rod_2 = len_2 # [m]
        self.density = 1000 # [kg/m^3]
        self.xz_rod = np.max([len_1, len_2])/10 # [m] height and width size rods
        self.xyz_cube = np.max([len_1, len_2])/5 # [m] size ground cube
        
        self.GetIinertialParams() # define inertial parameters
        self.CreateBodies(body_material) # initilize chrono body in system
        
    #---------------------------------
    # Calculate initial parameters and define how class parameters
    # Return dictionary of parameters
    #---------------------------------
    def GetIinertialParams(self):
        self.mass_rod_1 = self.xz_rod**2*self.len_rod_1*self.density
        self.mass_rod_2 = self.xz_rod**2*self.len_rod_2*self.density
        self.mass_cube = self.xyz_cube**3

        self.inertia_rod_1 = self.len_rod_1**2*self.mass_rod_1
        self.inertia_rod_2 = self.len_rod_2**2*self.mass_rod_2
        self.inertia_cube = 1/6*self.xyz_cube**2*self.mass_cube
        
        return {"m1":self.mass_rod_1,"m2":self.mass_rod_2,"I1":self.inertia_rod_1,"I2":self.inertia_rod_2}
    
    #---------------------------------
    # Initilize body in chrono system
    #---------------------------------
    def CreateBodies(self, body_material):
        
        # Initilize color asset for body
        color1 = chrono.ChColorAsset()
        color1.SetColor(chrono.ChColor(120, 0, 120))
        
        # Cube/ground
        self.body_cube = chrono.ChBody()
        self.body_cube.SetPos(chrono.ChVectorD(0,0,0)) # starting coordinates
        self.body_cube.SetMass(self.mass_cube)
        self.body_cube.SetInertiaXX(chrono.ChVectorD(self.inertia_cube,self.inertia_cube,self.inertia_cube))
        self.body_cube.SetBodyFixed(True) # fixed body in system
        self.body_cube.SetCollide(False) # turn off bodie's collision 
        
        
        # Create shape and add assets to body for visualisation box
        cube_shape = chrono.ChBoxShape()
        cube_shape.GetBoxGeometry().Size = chrono.ChVectorD(self.xyz_cube/2,self.xyz_cube/2,self.xyz_cube/2)
        self.body_cube.GetAssets().push_back(cube_shape)
        self.body_cube.GetAssets().push_back(color1)
        
        self.chrono_system.Add(self.body_cube)
        
        # Rod 1
        self.body_rod_1 = chrono.ChBody()
        self.body_rod_1.SetPos(chrono.ChVectorD(0, self.len_rod_1/2,0))
        self.body_rod_1.SetRot(chrono.Q_ROTATE_X_TO_Y) # Q_ROTATE_X_TO_Y - constant for rotating X axis to Y axis
        self.body_rod_1.SetMass(self.mass_rod_1)
        self.body_rod_1.SetInertiaXX(chrono.ChVectorD(self.inertia_rod_1,self.inertia_rod_1,self.inertia_rod_1))
        self.body_rod_1.SetBodyFixed(False)
        self.body_rod_1.SetCollide(False)

        color2 = chrono.ChColorAsset()
        color2.SetColor(chrono.ChColor(0,120,120))
        rod_1_shape = chrono.ChBoxShape()
        
        rod_1_shape.GetBoxGeometry().Size = chrono.ChVectorD(self.len_rod_1/2,self.xz_rod/2,self.xz_rod/2)
        self.body_rod_1.GetAssets().push_back(rod_1_shape)
        self.body_rod_1.GetAssets().push_back(color2)
        
        self.chrono_system.Add(self.body_rod_1)
        
        # Rod 2
        self.body_rod_2 = chrono.ChBody()
        self.body_rod_2.SetPos(chrono.ChVectorD(0, self.len_rod_2/2 + self.len_rod_1,0))
        self.body_rod_2.SetRot(chrono.Q_ROTATE_X_TO_Y)
        self.body_rod_2.SetMass(self.mass_rod_2)
        self.body_rod_2.SetInertiaXX(chrono.ChVectorD(self.inertia_rod_2,self.inertia_rod_2,self.inertia_rod_2))
        self.body_rod_2.SetBodyFixed(False)
        self.body_rod_2.SetCollide(False)

        color3 = chrono.ChColorAsset()
        color3.SetColor(chrono.ChColor(120,120,0))
        rod_2_shape = chrono.ChBoxShape()
        
        rod_2_shape.GetBoxGeometry().Size = chrono.ChVectorD(self.len_rod_2/2,self.xz_rod/2,self.xz_rod/2)
        self.body_rod_2.GetAssets().push_back(rod_2_shape)
        self.body_rod_2.GetAssets().push_back(color3)
        
        self.chrono_system.Add(self.body_rod_2)
    
    def AddJoints(self):

        # Ground to first rod
        self.joint_rev_gto1 = chrono.ChLinkMotorRotationTorque() # Rotion Motor. The torque is input
        frame_rev_1 = chrono.ChFrameD(chrono.ChVectorD(0,0,0),chrono.ChQuaternionD(1,0,0,0)) # Create representation of a 3D transform
        self.joint_rev_gto1.Initialize(self.body_rod_1,self.body_cube,frame_rev_1)
        # Add in system
        self.chrono_system.Add(self.joint_rev_gto1)

        # First to second rod
        self.motor_1to2 = chrono.ChLinkMotorRotationTorque()
        frame_rev_2 = chrono.ChFrameD(chrono.ChVectorD(0,self.len_rod_1,0))
        self.motor_1to2.Initialize(self.body_rod_2,self.body_rod_1,frame_rev_2)
        self.chrono_system.Add(self.motor_1to2)
    

    def Simulate(self, time = 20, time_step = 0.005):
        #Irrlicht module's object for setting runtime 3D visualisation
        #                                   chrono system, name of window, size of window
        myapplication = chrirr.ChIrrApp(self.chrono_system, 'Acrobot', chrirr.dimension2du(720,720))
        
        # Initilize skybox, camera, lights, shadow by default
        myapplication.AddTypicalSky()
        myapplication.AddTypicalCamera(chrirr.vector3df(0,0,1.7))
        myapplication.AddLightWithShadow(chrirr.vector3df(0,0,1.7),    # point
                                        chrirr.vector3df(0,0,0),    # aimpoint
                                        9,                 # radius (power)
                                        1,9,               # near, far
                                        60)                # angle of FOV
        myapplication.AssetBindAll()
        myapplication.AssetUpdateAll()
        myapplication.AddShadowAll()
        
        myapplication.SetTimestep(time_step)
        myapplication.SetTryRealtime(True)

        # List of output data
        arr_time = []
        ang1 = []
        ang2 = []
        omg1 = []
        omg2 = []
        input = []
        while(myapplication.GetDevice().run()):
            # LQR contol
            q = [self.joint_rev_gto1, self.motor_1to2]
            input = Feedback(q)
            
            # Create disturbance to start time
            if self.chrono_system.GetChTime() < 0.2:
                self.motor_1to2.SetTorqueFunction(chrono.ChFunction_Const(1))
            else:
                # Shape input signal for second motor
                self.motor_1to2.SetTorqueFunction(chrono.ChFunction_Const(input))
            
            # Add current data in list
            arr_time.append(self.chrono_system.GetChTime())
            ang1.append(q[0].GetMotorRot())
            ang2.append(q[1].GetMotorRot())
            omg1.append(q[0].GetMotorRot_dt())
            omg2.append(q[1].GetMotorRot_dt())
            
            
            myapplication.BeginScene(True, True, chrirr.SColor(160,125,125,125))
            myapplication.DrawAll()
            
            # Visualise frames and links
            chrirr.drawAllLinkframes(self.chrono_system, myapplication.GetVideoDriver(),1)
            chrirr.drawAllLinks(self.chrono_system, myapplication.GetVideoDriver(),1)
            
            myapplication.DoStep()
            myapplication.EndScene()
            # Condition stop simulate
            if self.chrono_system.GetChTime() > time:
                myapplication.GetDevice().closeDevice()
        return {'time': arr_time, 'angle_1' : ang1, 'angle_2': ang2, 'omega_1': omg1, 'omega_2':omg2}
