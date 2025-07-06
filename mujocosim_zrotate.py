import mujoco
import mujoco.viewer
import os
import time
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R

Lock = threading.Lock()
class SimMain():
    def __init__(self):
        self.actuators = ["j0", "j1", "j2", "j3", "j4", "j5", "j6"]
        self.GAINS={
            'Kp': 500.0,  # Proportional gain
            'Ki': 0,  # Integral gain
            'Kd': 100,  # Derivative gain
        }
        self.isfallen = False
        self.isfallen_prev = False
        self.largeangleerror = False
        self.onland = False
        self.commands = {}
        self.positions = {}
        self.velocities = {}
        self.forcelimits = {}
        self.speedlimits = {} #一旦使わないと思うけど一応定義
        self.powerlimits = {}
        self.actual_forces = {}
        self.rupos = [0,0,1]
        self.ruquat = [0,0,0,1]  # Quaternion representing no rotation
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
        # zrotate
        start_time = time.time()
        ctrl_period = 0.01  # Control loop period in seconds
        integral_yaw_error = 0
        Velgains = {
            'Kp': 50.0,  # Proportional gain
            'Ki': 1,  # Integral gain
        }

        while True:
            simtime = time.time() - start_time
            # MARK: Orientation control
            # z軸を中心に回転させる
            if self.onland:
                # 着地しているときは速度を0にする速度制御をする
                print("Setting Vel to 0")
                vel = (self.velocities["j5"] + self.velocities["j6"]) / 2.0
                torque = - (vel * Velgains['Kp'] + integral_yaw_error * Velgains['Ki'])  # PI制御
                self.commands["j5"] = torque
                self.commands["j6"] = torque
                pass
            else:
                integral_yaw_error = 0
                torque = 10
                print("Applying Constant Torque")
                self.commands["j5"] = torque
                self.commands["j6"] = torque
            time.sleep(ctrl_period)  # Sleep to maintain control loop period
            pass

    def control_loop_jump(self):
        start_time = time.time()
        ctrl_period = 0.01  # Control loop period in seconds
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
                self.onland = True
                landing_time_accumulation += ctrl_period
                if landing_time_accumulation > 0.2:
                    # 着地後0.5秒立ってからジャンプ
                    self.commands["j0"] = self.forcelimits["j0"] 
                if simtime % 1 < ctrl_period:
                    print(f"centerpos: {self.centerpos}")
            else:
                self.isfallen = False
                self.onland = False
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
