import mujoco
import mujoco.viewer
import os
import time
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Point,Twist

Lock = threading.Lock()
class SimMain():
    def __init__(self):

        # Initialize ROS node
        # rospy.init_node('mujocosim_node', anonymous=True)
        # rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        self.cmd_vel = Twist()
        self.RW_Inertia = 3.84 # MARK:⚠️これはxmlと一致させる必要あり

        self.actuators = ["j0", "j1", "j2", "j3", "j4", "j5", "j6"]
        self.GAINS={
            'Kp': 500.0,  # Proportional gain
            'Ki': 0,  # Integral gain
            'Kd': 100,  # Derivative gain
        }
        self.isfallen = False
        self.isfallen_prev = False
        self.largeangleerror = False
        
        self.commands = {}
        self.positions = {}
        self.velocities = {}
        self.forcelimits = {}
        self.speedlimits = {} #一旦使わないと思うけど一応定義
        self.powerlimits = {}
        self.actual_forces = {}
        self.rupos = [0,0,1]
        self.ruquat = [0,0,0,1]  # Quaternion representing no rotation
        self.ruquat_prev = [0,0,0,1]  # Previous quaternion for comparison
        self.centerpos = [0,0,0]
        self.centerquat = [0,0,0,1]  # Quaternion representing no rotation
        maxforce = 10**3
        maxspeed = 10**2
        maxspeed_prismatic = 10**12
        maxforce_prismatic = 10**3
        maxpower = 2000
        for actuator in self.actuators:
            self.commands[actuator] = 0.0
            self.positions[actuator] = 0.0
            self.velocities[actuator] = 0.0
            self.actual_forces[actuator] = 0.0
            self.forcelimits[actuator] =maxforce if actuator != "j0" else maxforce_prismatic
            self.powerlimits[actuator] = maxpower
            self.speedlimits[actuator] = maxspeed if actuator != "j0" else maxspeed_prismatic

        
        self.mujocosim_thread = threading.Thread(target=self.mujocosim)        
        self.mujocosim_thread.start()
        self.control_loop_orientation_thread = threading.Thread(target=self.control_loop_orientation)
        self.control_loop_orientation_thread.start()
        self.control_loop_jump_thread = threading.Thread(target=self.control_loop_jump)
        self.control_loop_jump_thread.start()
        self.mujocosim_thread.join()
        self.control_loop_orientation_thread.join()
        self.control_loop_jump_thread.join()
        pass
    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def apply_force_under_limit(self,joint):
        #with Lock:
        vel = self.velocities[joint]
        force = self.commands[joint]
        #負の仕事はいくらでも許容可能
        if vel * force <0:
            return force
        #正の仕事(forceとvelが同符号の場合

        # 速度が小さいときは力の制限を適用
        if abs(vel) < self.powerlimits[joint] / self.forcelimits[joint]:
            return np.clip(force, -self.forcelimits[joint], self.forcelimits[joint])
        else:
            return np.clip(force, -self.powerlimits[joint]/vel, self.powerlimits[joint]/vel)

    def control_loop_orientation(self):
        start_time = time.time()
        ctrl_period = 0.01  # Control loop period in seconds
        self.target_angles = np.array([np.deg2rad(0), np.deg2rad(35),0])  # Target angles for pitch and roll in radians
        prev_angles_error = np.zeros(3)  # Previous error for angles
        integral_angles_error = np.zeros(3)  # Integral error for angles
        velgain = 20

        while True:
            simtime = time.time() - start_time
            if self.isfallen and abs(self.velocities["j3"] + self.velocities["j4"])/2 > 20:
                # MARK: 倒れた状態では一旦速度を0にするP制御を書け続ける
                self.commands["j1"] = - self.velocities["j1"] * velgain
                self.commands["j2"] = - self.velocities["j2"] * velgain
                self.commands["j3"] = - self.velocities["j3"] * velgain
                self.commands["j4"] = - self.velocities["j4"] * velgain

                pass
            else:
                # MARK: Orientation control
                # with Lock:
                ruquat = self.ruquat
                rotation = R.from_quat(ruquat)
                angles = np.array([rotation.as_euler('xyz', degrees=False)[0],
                                   rotation.as_euler('yzx', degrees=False)[0],
                                   rotation.as_euler('zxy', degrees=False)[0]
                                   ])  # Get pitch and roll angles in radians
                angles_error = self.target_angles - angles
                print(f"angles_error: {angles_error}")
                fall_threshold = np.deg2rad(36)  # 角度の閾値
                if angles_error[0] < -fall_threshold or angles_error[1] < -fall_threshold: # 直立は倒れている判定ではないので、absはつけない
                    # 倒れているとみなす
                    self.largeangleerror = True
                integral_angles_error += angles_error * ctrl_period
                derivative_angles_error = (angles_error - prev_angles_error) / ctrl_period
                prev_angles_error = angles_error

                # PID control for angles
                angles_control = (self.GAINS['Kp'] * angles_error +
                            self.GAINS['Ki'] * integral_angles_error +
                            self.GAINS['Kd'] * derivative_angles_error)

                # Adjustments: リアクション・ホイール自体がピッチに対してどっち側に回るか
                angles_control_adjusted = angles_control
                if simtime % 1 < ctrl_period:
                    # Debugging output
                    pass
                # Apply the control to the actuators 
                # with Lock:
                self.commands["j1"] = angles_control_adjusted[0]
                self.commands["j2"] = angles_control_adjusted[0]
                self.commands["j3"] = angles_control_adjusted[1]
                self.commands["j4"] = angles_control_adjusted[1]
                self.commands["j5"] = angles_control_adjusted[2]
                self.commands["j6"] = angles_control_adjusted[2]

                # # MARK: Z-axis Gyro Compensation
                # angles_derivative = R.from_matrix(R.from_quat(self.ruquat_prev).as_matrix() @ rotation.as_matrix()).as_euler('xyz', degrees=False)[:2]/ctrl_period  # Get the derivative of angles in radians

                # gyrotorque = 0
                # gyrotorque_x = - self.RW_Inertia * (self.velocities["j1"] + self.velocities["j2"]) * angles_derivative[1]
                # gyrotorque_y =  self.RW_Inertia * (self.velocities["j3"] + self.velocities["j4"]) * angles_derivative[0]
                # gyrotorque = gyrotorque_x + gyrotorque_y
                # gyrocomp = -gyrotorque 
                # self.commands["j5"] = gyrocomp
                # self.commands["j6"] = gyrocomp
                # print("TEST POINT")
            self.isfallen_prev = self.isfallen
            self.ruquat_prev = self.ruquat.copy()  # Save the previous quaternion for comparison
            time.sleep(ctrl_period)  # Sleep to maintain control loop period
            pass
    
    def control_loop_jump(self):
        start_time = time.time()
        ctrl_period = 0.01  # Control loop period in seconds
        onland = False
        land_height_threshold = 1.3
        fallen_height_threshold = 0.5
        landing_time_accumulation = 0.0
        while True:
            simtime = time.time() - start_time
            # MARK: Jump control
            # with Lock:
            if self.centerpos[2] < fallen_height_threshold or self.largeangleerror:
                # 倒れている
                self.isfallen = True
                self.commands["j0"] = - self.forcelimits["j0"] 
                if simtime % 1 < ctrl_period:
                    print(f"Fallen! centerpos: {self.centerpos}")
            elif self.centerpos[2] < land_height_threshold:
                # 着地状態とする
                self.isfallen = False
                onland = True
                landing_time_accumulation += ctrl_period
                if landing_time_accumulation > 0.2:
                    # 着地後0.5秒立ってからジャンプ
                    self.commands["j0"] = self.forcelimits["j0"] 
                if simtime % 1 < ctrl_period:
                    print(f"centerpos: {self.centerpos}")
            else:
                self.isfallen = False
                # thresh超えてたらジャンプ状態とする.
                self.commands["j0"] = -1.0  # Reset prismatic joint force
                landing_time_accumulation = 0.0
                if simtime % 1 < ctrl_period:
                    print(f"Jumping! centerpos: {self.centerpos}")

            time.sleep(ctrl_period)  # Sleep to maintain control loop period




    def mujocosim(self):
        m = mujoco.MjModel.from_xml_path(os.getcwd()+'/DynaBox.xml')
        d = mujoco.MjData(m)
        m.opt.timestep = 0.002  # Set the simulation timestep ココを雑にすると物体貫通とかめんどいことが起きる
        with mujoco.viewer.launch_passive(m, d) as viewer:
            #カメラ用のコンテキストをセット
            #context = mujoco.MjRenderContextOffscreen(m)
            #cam_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "eye")


            while viewer.is_running():
                step_start = time.time()
                # with Lock:
                self.rupos = d.body("REACTION_UNIT_BASE").xpos
                quat_temp = list(d.body("REACTION_UNIT_BASE").xquat)
                # with Lock:
                self.ruquat = quat_temp[1:] + [quat_temp[0]]  # Convert w,x,y,z order to x,y,z,w order
                #xposとxquatのようなxのついたものはグローバル座標での値らしい

                self.centerpos = d.body("CENTER").xpos
                centerquad_temp = list(d.body("CENTER").xquat)
                self.centerquat = centerquad_temp[1:] + [centerquad_temp[0]]
                for j in self.actuators:
                    # with Lock:
                    d.actuator(j).ctrl = self.apply_force_under_limit(j)
                    self.actual_forces[j] = d.actuator(j).force[0]
                    self.positions[j] = d.actuator(j).length[0]
                    self.velocities[j] = d.actuator(j).velocity[0]

                mujoco.mj_step(m, d)
                viewer.sync()

                # Rudimentary time keeping, will drift relative to wall clock.
                time_until_next_step = m.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                # print("TEST POINT:mujocosim")

if __name__ == "__main__":
    sim = SimMain()
