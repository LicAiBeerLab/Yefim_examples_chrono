import pychrono.core as chr
import pychrono.irrlicht as chrirr
import numpy as np

 
class MechanismSystem():
    def __init__(self, in_system, len_link_2, length_BC):
        
        self.chr_system = in_system
        
        self.link_2 = len_link_2 # [m]
        self.l_BC = length_BC # [m]
        
        self.y_AB = self.l_BC/2 # [m]
        self.x_AB = self.link_2/2 # [m]
        self.link_1 = self.y_AB*np.sqrt(2) # [m]
        self.link_5 = self.link_1/2 # [m]
        self.link_4 = self.l_BC # [m]
        
        self.density = 7826 # kg/m^3 (Сталь45)
        self.radius_link = min([self.link_1, self.link_2, self.link_5])/10 #[m] cut size link

    def CreateBodies(self):
        color_mag = chr.ChColorAsset()
        color_mag.SetColor(chr.ChColor(200, 0, 200)) # magenta
        
        color_cyan = chr.ChColorAsset()
        color_cyan.SetColor(chr.ChColor(0, 255, 120)) # cyan
        
        color_yellow = chr.ChColorAsset()
        color_yellow.SetColor(chr.ChColor(200, 200, 0)) # yellow
        
        color_green = chr.ChColorAsset()
        color_green.SetColor(chr.ChColor(0, 200, 0)) #  green
        
        color_red = chr.ChColorAsset()
        color_red.SetColor(chr.ChColor(200, 0, 0)) #  red
        
        
        
        self.body_link_1 = chr.ChBodyEasyCylinder(self.radius_link, self.link_1, self.density) # Class for quickly creating simple body (Cylinder, Box, Sphere and others)
        self.body_link_1.SetPos(chr.ChVectorD(0,0,0))
        self.body_link_1.SetRot(chr.Q_from_AngZ(np.arctan(1)))
        self.body_link_1.SetBodyFixed(True)
        self.body_link_1.AddAsset(color_mag)
        self.chr_system.Add(self.body_link_1)
        
        # create frame for next joint in first link's frame
        frame_joint_15 = chr.ChFrameMovingD()
        frame_joint_15.SetPos(chr.ChVectorD(0,self.link_1/2,0))
        frame_joint_15.SetRot(chr.Q_from_AngZ(-chr.CH_C_PI_2))
        # transform relative to world frame (>> - operator LEFT TO RIGHT transformations, * - operator RIGHT TO LEFT transformations)
        self.frame_05 =  frame_joint_15 >> self.body_link_1.GetFrame_COG_to_abs() 

        # Frame for fifth link
        link_COG_5 = chr.ChVectorD(0,self.link_5/2,0)
        vec_05 = link_COG_5 >> self.frame_05 

        self.body_link_5 = chr.ChBodyEasyCylinder(self.radius_link,self.link_5,self.density)
        self.body_link_5.SetPos(vec_05)
        self.body_link_5.SetRot(self.frame_05.GetRot())
        self.body_link_5.SetBodyFixed(True)
        self.body_link_5.AddAsset(color_cyan)
        self.chr_system.Add(self.body_link_5)

        # same operations for sixth link
        frame_joint_16 = chr.ChFrameMovingD()
        frame_joint_16.SetPos(chr.ChVectorD(0,-self.link_1/2,0))
        frame_joint_16.SetRot(chr.Q_from_AngZ(chr.CH_C_PI_2))
        self.frame_06 =  frame_joint_16 >> self.body_link_1.GetFrame_COG_to_abs() 
        vec_06 = link_COG_5 >> self.frame_06
        
        self.body_link_6 = chr.ChBodyEasyCylinder(self.radius_link,self.link_5, self.density)
        self.body_link_6.SetPos(vec_06)
        self.body_link_6.SetRot(self.frame_06.GetRot())
        self.body_link_6.SetBodyFixed(True)
        self.body_link_6.AddAsset(color_yellow)
        self.chr_system.Add(self.body_link_6)
        
        self.body_link_2 = chr.ChBodyEasyCylinder(self.radius_link,self.link_2,self.density)
        self.body_link_2.SetPos(chr.ChVectorD(0,self.y_AB,0))
        self.body_link_2.SetRot(chr.Q_ROTATE_X_TO_Y)
        self.body_link_2.SetBodyFixed(True)
        self.body_link_2.AddAsset(color_green)
        self.chr_system.Add(self.body_link_2)
        
        
        self.body_link_3 = chr.ChBodyEasyCylinder(self.radius_link,self.link_2,self.density)
        self.body_link_3.SetPos(chr.ChVectorD(0,-self.y_AB,0))
        self.body_link_3.SetRot(chr.Q_ROTATE_X_TO_Y)
        self.body_link_3.SetBodyFixed(True)
        self.body_link_3.AddAsset(color_red)
        self.chr_system.Add(self.body_link_3)
        
        
        
    def AddJoints(self):
        
        #ChLinkRevolute - simple uncontrollable revolute joint without sensor by default 
        joint_15 = chr.ChLinkRevolute()
        joint_15.Initialize(self.body_link_1,self.body_link_5,self.frame_05)
        self.body_link_5.SetBodyFixed(False)
        self.chr_system.Add(joint_15)
        
        joint_52 = chr.ChLinkRevolute()
        joint_52.Initialize(self.body_link_5,self.body_link_2,
                            chr.ChFrameD(chr.ChVectorD(0,self.y_AB,0)))
        self.body_link_2.SetBodyFixed(False)
        self.chr_system.Add(joint_52)
        
        ground = chr.ChBodyEasyBox(0.001,0.001,0.001,100,False)
        ground.SetBodyFixed(True)
        self.chr_system.Add(ground)
        
        joint_02 = chr.ChLinkRevolute()
        joint_02.Initialize(ground,self.body_link_2,
                            chr.ChFrameD(chr.ChVectorD(-self.link_2/2,self.l_BC/2,0)))
        self.chr_system.Add(joint_02)
        
        joint_16 = chr.ChLinkRevolute()
        joint_16.Initialize(self.body_link_1,self.body_link_6,self.frame_06)
        self.body_link_6.SetBodyFixed(False)
        self.chr_system.Add(joint_16)
        
        joint_63 = chr.ChLinkRevolute()
        joint_63.Initialize(self.body_link_6,self.body_link_3,
                            chr.ChFrameD(chr.ChVectorD(0,-self.y_AB,0)))
        self.body_link_3.SetBodyFixed(False)
        self.chr_system.Add(joint_63)
        
        
        joint_03 = chr.ChLinkRevolute()
        joint_03.Initialize(ground,self.body_link_3,
                            chr.ChFrameD(chr.ChVectorD(-self.link_2/2,-self.y_AB,0)))
        self.chr_system.Add(joint_03)
        
        self.motorA = chr.ChLinkMotorRotationTorque()
        self.motorA.Initialize(self.body_link_1,ground,
                          chr.ChFrameD(chr.ChVectorD(0,0,0)))
        
        # Function for shape motor's input 
        func_torque =chr.ChFunction_Ramp(0,-2)
        self.motorA.SetTorqueFunction(func_torque)
        self.body_link_1.SetBodyFixed(False)
        self.chr_system.Add(self.motorA)
        
        # Spring parameters
        rel_len_spring = self.l_BC #[m] - length of resting
        spring_coeff = 200 #[H/m] - spring constant
        damping_coeff = 1 # - coefficient of damping

        spring = chr.ChLinkTSDA() # Spring link
        spring.Initialize(self.body_link_2, self.body_link_3,
                          True, # Position are relative
                          chr.ChVectorD(0,-0.8*self.link_2/2,0), # position spring connection relative link 2 
                          chr.ChVectorD(0,-0.8*self.link_2/2,0),
                          True, # Auto resting length
                          rel_len_spring)
        spring.SetSpringCoefficient(spring_coeff)
        spring.SetDampingCoefficient(damping_coeff)
        
        # Visualise spring
        color_spring = chr.ChColorAsset()
        color_spring.SetColor(chr.ChColor(0,0.5,0))
        spring.AddAsset(color_spring)
        spring.AddAsset(chr.ChPointPointSpring(0.01,80,15))
        
        self.chr_system.AddLink(spring)
        
        
        
    def Simulate(self, time_stop=6, time_step=0.001):
        myapplication = chrirr.ChIrrApp(self.chr_system, 'Articualted-lever mechanism with sptink link',
                                        chrirr.dimension2du(1280,720))

        myapplication.AddTypicalSky()
        myapplication.AddTypicalCamera(chrirr.vector3df(0,0,0.5))
        myapplication.AddLightWithShadow(chrirr.vector3df(0,0,1.7),    # point
                                        chrirr.vector3df(0,0,0),    # aimpoint
                                        9,                 # radius (power)
                                        1,9,               # near, far
                                        60)                # angle of FOV
        myapplication.AssetBindAll()
        myapplication.AssetUpdateAll()
        myapplication.AddShadowAll()
        
        time = []
        ang_motorA = []
        omg_motorA = []
        
        myapplication.SetTimestep(time_step)
        myapplication.SetTryRealtime(True)
        while(myapplication.GetDevice().run()):
            myapplication.BeginScene()
            myapplication.DrawAll()
            
            chrirr.drawAllLinkframes(self.chr_system, myapplication.GetVideoDriver(),1)

            time.append(self.chr_system.GetChTime())
            ang_motorA.append(self.motorA.GetMotorRot())
            omg_motorA.append(self.motorA.GetMotorRot_dt())
            
            if np.mod(self.chr_system.GetChTime(),3) > 1.5:
                self.motorA.SetTorqueFunction(chr.ChFunction_Const(0))
            else :
                func_torque =chr.ChFunction_Ramp(0,-0.9)
                self.motorA.SetTorqueFunction(func_torque)    
                
            myapplication.DoStep()
            myapplication.EndScene()
            if self.chr_system.GetChTime() > time_stop:
                myapplication.GetDevice().closeDevice()
        return time, ang_motorA, omg_motorA