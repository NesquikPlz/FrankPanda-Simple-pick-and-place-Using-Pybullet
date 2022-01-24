import pybullet as p
import pybullet_data
import math
import numpy as np


class PandaEnv():
    def __init__(self):
        self.step_counter = 0
        p.connect(p.GUI)
        p.resetDebugVisualizerCamera(cameraDistance=0.7, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2])

    def duck_position(self, i):
        self.duck = i - 1
        duckpos, _ = p.getBasePositionAndOrientation(self.objectUid[self.duck])
        return duckpos

    def step(self, action, process, duck):
        self.duck = duck
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        orientation = p.getQuaternionFromEuler([0.,-math.pi,math.pi/2.])
        dv = 0.0022
        dx = action[0] * dv
        dy = action[1] * dv
        dz = action[2] * dv
        fingers = action[3]

        currentPose = p.getLinkState(self.pandaUid, 11)
        currentPosition = currentPose[0]
        print('current pose : ', currentPosition)
        newPosition = [currentPosition[0] + dx,
                       currentPosition[1] + dy,
                       currentPosition[2] + dz]
        jointPoses = p.calculateInverseKinematics(self.pandaUid,11,newPosition, orientation)[0:7]

        p.setJointMotorControlArray(self.pandaUid, list(range(7))+[9,10], p.POSITION_CONTROL, list(jointPoses)+2*[fingers])

        p.stepSimulation()
        state_goal_object, _ = p.getBasePositionAndOrientation(self.objectUid[self.duck])

        local_goal_object = self.objectUid[self.duck]
        if process == 1 or process == 2 : # move to tray
            local_goal_object = self.trayUid

        state_object, _ = p.getBasePositionAndOrientation(local_goal_object)
        state_robot = p.getLinkState(self.pandaUid, 11)[0]
        state_fingers = (p.getJointState(self.pandaUid,9)[0], p.getJointState(self.pandaUid, 10)[0])

        print('state_object : ', state_goal_object)
        print('process : ', process)
        if process == 0 and state_goal_object[2]>0.4: # picked a duck successfully
            done = True
        elif process == 1 and state_goal_object[0]>0.65 : # moved a duck above the tray
            done = True
        elif process == 2 and state_goal_object[0]>0.6 and state_goal_object[2]<0.15: # release the gripper
            done = True
        elif process == 3 : # put a duck on the tray
            done = True
        elif process == 4 and currentPosition[2]>0.3 and currentPosition[0]<0.4: # initialized panda's pos
            done = True
        else:
            done = False

        self.step_counter += 1

        info = {'object_position': state_object}
        self.observation = state_robot + state_fingers
        reward = 0
        return np.array(self.observation).astype(np.float32), reward, done, info

    def reset(self):
        duckcount = 3
        rotate = [0, 0.1, 0.6, 0]
        dis = 0.45 / duckcount

        self.step_counter = 0
        p.resetSimulation()
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0) # we will enable rendering after we loaded everything
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)

        self.pandaUid = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
        self.tableUid = p.loadURDF("table/table.urdf", basePosition=[0.5, 0, -0.65])
        self.plainID = p.loadURDF("plane.urdf", basePosition=[0, 0, -0.65])
        self.trayUid = p.loadURDF("tray/traybox.urdf", basePosition=[1., 0, 0])
        self.objectUid= []

        '''reset panda's poses'''
        rest_poses = [0, -0.215, 0, -2.57, 0, 2.356, 2.356, 0.08, 0.08]
        for i in range(7):
            p.resetJointState(self.pandaUid, i, rest_poses[i])
        p.resetJointState(self.pandaUid, 9, 0.08)
        p.resetJointState(self.pandaUid, 10, 0.08)

        for i in range(duckcount):
            inipos = [0.55 - dis * i, 0, 0]
            self.objectUid.append(p.loadURDF("lego/lego.urdf", basePosition = inipos, baseOrientation= rotate, globalScaling = 1.5))

        state_robot = p.getLinkState(self.pandaUid, 11)[0]
        state_fingers = (p.getJointState(self.pandaUid,9)[0], p.getJointState(self.pandaUid, 10)[0])
        self.observation = state_robot + state_fingers
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)

        return np.array(self.observation).astype(np.float32)


    def render(self):
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.7,0,0.05],
                                                            distance=.7,
                                                            yaw=90,
                                                            pitch=-70,
                                                            roll=0,
                                                            upAxisIndex=2)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                     aspect=float(960) /720,
                                                     nearVal=0.1,
                                                     farVal=100.0)
        (_, _, px, _, _) = p.getCameraImage(width=960,
                                              height=720,
                                              viewMatrix=view_matrix,
                                              projectionMatrix=proj_matrix,
                                              renderer=p.ER_BULLET_HARDWARE_OPENGL)

        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (720,960, 4))

        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def _get_state(self):
        return self.observation

    def close(self):
        p.disconnect()
