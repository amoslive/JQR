# %%
import time
import matplotlib.pyplot as plt
import mujoco 
import mujoco.viewer
import numpy as np
import math
from math import cos,sin,atan,sqrt
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
import scipy.io
import mediapy as media 
import control as ct 
from control import lqr
import ecbc_toolbox as ecbc
from bike_model import WheelieGround
pi = np.pi
print("mujoco version:", mujoco.__version__)

# %%

ct_time_step = 0.005 # control time step
ct_time = 0.0 # next control time
cur_time = 0.0 # current time
mujoco.set_mjcb_control(None)
model = mujoco.MjModel.from_xml_path("model/scene_ramp.xml")
data = mujoco.MjData(model)

body_camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "body_camera")
print(f"body_camera_id: {body_camera_id}")

# set initial pose
pole_len = 0.3
wheel_radius = 0.09
wst0 = -0.0/180.0*pi  # initial waist angle
th0 = - pi/2.0 + 0.1  # initial tilt angle
pole_pos_0 = np.array([0, 0, wheel_radius])
pole_quat_0 = Rot.from_euler('xyz', [0, th0, 0]).as_quat() # note: SciPy 'xyz' uses fixed axes; MuJoCo 'XYZ' uses fixed axes
pole_quat_0 = np.concatenate((pole_quat_0[-1:], pole_quat_0[:-1])) # note: older SciPy stores quaternion scalar part at the end
data.qpos[:7] = np.concatenate((pole_pos_0, pole_quat_0))
data.qpos[8] = wst0
mujoco.mj_step(model, data)

mb = 1.70488 # 1.70208
mf = 5.91501 # 6.27033
mrw = 0.43239 # 0.41106
mfw = 0.35533
M = mb+mf+mrw

lb = 0.3
lf = 0.3
r = 0.09

db = 0.14306 # 0.14332
df = 0.16170 # 0.16954
Ib = 0.06089 # 0.02664
If = 0.20020 # 0.05828
Irw = 0.00140 # 0.00139
Ifw = 0.00135

g = 9.8
mu = 0.5

frw = 0
fwst = 0

c_tilt_pos_ref = 0
c_tilt_vel_ref = 0
waist_pos_ref = wst0
wheel_pos_ref = 0
wheel_vel_ref = 0
xref = np.array([0.0, 0.0, 0.0, 0.0])

maxtime=3 

c_pos=[]
cnt=0
#print(ode_fr)

waist_car = WheelieGround(ct_time_step, waist_pos_ref)
# obsv = ecbc.LinearObserver(waist_car.sys, 'place_distinct_poles', True, poles_obs=[-30,-30,-40,-40,-3])
obsv = ecbc.LinearObserver(waist_car.sys, 'lqr', True, Qo=np.diag([100, 100, 100, 1000, 1000]), Ro=np.diag([1, 1, 1]))
ctrl = ecbc.LinearController(waist_car.sys, 'place_distinct_poles', poles_ctr=[-6.5, -6.0, -3.5])

lpf_dist_pitch = ecbc.LowpassFilter(ct_time_step, 1)
lpf_dist_whtor = ecbc.LowpassFilter(ct_time_step, 1)
last_act_ctrl = np.array([[0]])
ramp_pitch_dist = 0.0
ramp_whtor_dist = 0.0
last_whtor_dist = 0.0

# %%
def controller(model, data):
    global c_pos, cnt
    global ct_time, cur_time
    global waist_pos_ref, x_ref
    global last_act_ctrl
    global ramp_pitch_dist, ramp_whtor_dist
    global last_whtor_dist
    # current time
    cur_time = data.time
    if data.time >= ct_time: # run at a fixed interval (works for both active and passive stepping)
        ct_time += ct_time_step
        cnt += 1
        # current time
        # print(f"cur_time:{cur_time:.2f}")
        # current sensor readings
        waist_joint_pos=data.sensor("pos_waist_joint").data[0].copy()
        waist_joint_vel=data.sensor("vel_waist_joint").data[0].copy()
        back_wheel_joint_pos=data.sensor("pos_back_joint").data[0].copy()
        back_wheel_joint_vel=data.sensor("vel_back_joint").data[0].copy()
        gyro = data.sensor("gyro").data
        y_temp=data.sensor("pos_back_wheel_center").data[2]
        orient = data.sensor("orient").data
        orient = np.concatenate((orient[-3:], orient[:-3])) # note: older SciPy stores quaternion scalar part at the end
        euler_angles = Rot.from_quat(orient).as_euler('xyz', degrees=False)
        # planar coordinates: forward x, up y, tilt angle
        tilt_pos = euler_angles[0] - pi/2.0 + 0.0
        tilt_vel = gyro[0]
        # print(f"tilt_pos:{tilt_pos*180/pi:.2f}, tilt_vel:{tilt_vel*180/pi:.2f}")

        #compute center of mass
        mc = mb + mf + mfw
        xml = (mb*db + mf*(lb+df*cos(waist_joint_pos)) + mfw*(lb+lf*cos(waist_joint_pos)))/mc
        yml = (mb*0  + mf*df*sin(waist_joint_pos) + mfw*lf*sin(waist_joint_pos))/mc
        center_ang = math.atan2(yml, xml)
        lc = sqrt(pow(xml,2)+pow(yml,2))
        Ic = (Ib+If+Ifw+
              mb*(pow(db-xml,2)+pow(0-yml,2))+ 
              mf*(pow(lb+df*cos(waist_joint_pos)-xml,2)+pow(df*sin(waist_joint_pos)-yml,2)) + 
              mfw*(pow(lb+lf*cos(waist_joint_pos)-xml,2)+pow(lf*sin(waist_joint_pos)-yml,2)))
        center_ang = math.atan2(yml, xml)
        c_tilt_pos = tilt_pos + center_ang
        c_tilt_vel = tilt_vel + (mf*df+mfw*lf)/(mc*pow(lc,2))*(xml*cos(center_ang)+yml*sin(center_ang))*waist_joint_vel
        c_pos.append(c_tilt_pos)

        # lc2 = (mb*db + mf*(lb+df) + mfw*(lb+lf))/mc
        # Ic2 = Ib+If+Ifw+mb*db**2 + mf*(lb+df)**2 + mfw*(lb+lf)**2
        # print(f"lc:{lc:.3f}, lc2:{lc2:.3f}, center_ang:{center_ang*180/pi:.2f}, Ic:{Ic+mc*lc**2:.4f}, Ic2:{Ic2:.4f}")

        kpfb = 100
        kdfb = 100
        # I ddx =  tau = I*(kp * x + kd * dx)
        Ifwc = If + Ifw + mf*pow(df,2) + mfw*pow(lf,2)
        kwomega = 20.0
        kwzeta = 1.0
        fwst = Ifwc * (kwomega**2*(waist_pos_ref - waist_joint_pos) + 2*kwomega*kwzeta*(0 - waist_joint_vel))

        sigma1 = Ic*Irw + Ic*(mc+mrw)*pow(r,2) + Irw*mc*pow(lc,2) + mrw*pow(lc,2)*mc*pow(r,2)
        g = 9.81
        A = np.array([[0, 0, 1, 0],
                      [0, 0, 0, 1],
                      [mc*lc*(Irw*g+(mc+mrw)*g*r**2)/sigma1, 0, 0, 0],
                      [-(mc**2*g*lc*r*(lc+r) + mrw*g*lc*mc*r**2 + mc*g*lc*Irw)/sigma1, 0, 0, 0]])
        B = np.array([[0],
                      [0],
                      [-(Irw+(mc+mrw)*pow(r,2)+mc*lc*r)/sigma1],
                      [(Ic+Irw+(mc+mrw)*pow(r,2)+mc*pow(lc,2)+2*mc*lc*r)/sigma1]])
        
        act_ecbc_ym = np.array([c_tilt_pos, c_tilt_vel, back_wheel_joint_vel]).reshape((3,1))
        if ct_time == ct_time_step:
            obsv.resetState(np.array([c_tilt_pos, c_tilt_vel, back_wheel_joint_vel, 0, 0]).reshape((5,1)))
            est_state = np.zeros((3,1))
            est_dist = np.zeros((2,1))
        else:
            est_state, est_dist = obsv.stepAndGetEstimation(last_act_ctrl, act_ecbc_ym)
            # if est_dist[1,0] > 1.0 and last_whtor_dist < 1.0:
            #     ramp_ang = 20.0/180.0*pi
            #     ramp_whtor_dist = (mc+mrw)*g*sin(ramp_ang)*r  # ramp torque
            #     ramp_pitch_dist = ramp_whtor_dist / (mc*g*lc)
            #     obsv.resetState(np.array([c_tilt_pos, c_tilt_vel, back_wheel_joint_vel, ramp_pitch_dist, ramp_whtor_dist]).reshape((5,1)))
            #     est_dist = np.array([[ramp_pitch_dist], [ramp_whtor_dist]])
            #     lpf_dist_pitch.xout = -ramp_pitch_dist
            #     lpf_dist_whtor.xout = -ramp_whtor_dist
            last_whtor_dist = est_dist[1,0]

        ref_rear_vel = -6.0
        act_ecbc_yr = np.array([ref_rear_vel]).reshape((1,1))
        ref_state = np.array([[0.0, 0.0, ref_rear_vel]]).reshape(3,1)
        enable_comp = True
        if enable_comp and ct_time > 1.0:
            equ_state, equ_control = waist_car.sys.findEquilibrium(est_dist, act_ecbc_yr)
            equ_state[0,0] = lpf_dist_pitch.stepAndGetOutput(equ_state[0,0])
            equ_control[0,0] = lpf_dist_whtor.stepAndGetOutput(equ_control[0,0])
            cmd_rear_tor = ctrl.stepAndGetControl(act_ecbc_ym, equ_state, ct_time)[0,0] + equ_control[0,0]
            # cmd_rear_tor = ctrl.stepAndGetControl(act_ecbc_ym, equ_state, ct_time)[0,0]
            print(f"equ_state: {equ_state.ravel()}, equ_control: {equ_control.ravel()}")
            print(f"est_dist: {est_dist.ravel()}")
        else:
            cmd_rear_tor = ctrl.stepAndGetControl(act_ecbc_ym, ref_state, ct_time)[0,0]

        frw = cmd_rear_tor
        last_act_ctrl = np.array([[frw]])

        # if back_wheel_joint_vel > 0.1:
        #     frw += 0.4
        # elif back_wheel_joint_vel < -0.1:
        #     frw -= 0.4
        if waist_joint_vel > 0.05:
            fwst += 0.4
        elif waist_joint_vel < -0.05:
            fwst -= 0.4

        # if cnt % 100 == 0:
        #     print(f"poles: {poles}, K: {K}")
        #     print(f"x_ref: {xref}, x: {x}")
        #     print(f"center_ang: {center_ang*180/pi:.2f}, lc: {lc:.3f}")

    
        # write control commands to the simulator
        data.ctrl[0] = frw
        data.ctrl[1] = fwst

# %%
# set control callback
mujoco.set_mjcb_control(controller)
# mujoco.set_mjcb_control(drift_controller)
# mujoco.set_mjcb_control(None)
# model.opt.disableactuator = 7

duration = maxtime  # simulation time(seconds)
framerate = 30  # (Hz)
playbakerate = 1

frames = []
height = 320
width = 480
flag=0

#$$$$$$$$  usage1 Start  launch MuJoCo Viewer  $$$$$$$$
mujoco.viewer.launch(model, data)
#$$$$$$$$  usage1 END  $$$$$$$$

#$$$$$$$$  usage2 Start  record video  $$$$$$$$
# with mujoco.Renderer(model, height, width) as renderer:
#   while data.time < duration:
#     mujoco.mj_step(model, data)    
#     if len(frames) < data.time * framerate / playbakerate:   
#       renderer.update_scene(data,'camera')
#       pixels = renderer.render()
#       frames.append(pixels)

# media.show_video(frames, fps=framerate)
#$$$$$$$$  usage2 END  $$$$$$$$

#media.write_video("jump_high_1.mp4", frames, fps=framerate)



