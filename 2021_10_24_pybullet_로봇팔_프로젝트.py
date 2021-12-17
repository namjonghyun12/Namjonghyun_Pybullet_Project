import os
import pybullet as p
import pybullet_data
import time
import math

# distance(x, y): compute the Euclidean distance between 2D points x and y
def distance1D(point_one, point_two):
    return ((point_one-point_two) ** 2) ** 0.5

def distance2D(point_one, point_two):
    return ((point_one[0] - point_two[0]) ** 2 +
            (point_one[1] - point_two[1]) ** 2) ** 0.5

def distance3D(point_one, point_two):
    return ((point_one[0] - point_two[0]) ** 2 +
            (point_one[1] - point_two[1]) ** 2 +
            (point_one[2] - point_two[2]) ** 2) ** 0.5

duration = 3000
stepsize = 1e-3
realtime = 1
jaw_size = 0.08
close_dist = 0.035

position_control_gain_p = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
position_control_gain_d = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
max_torque = [100, 100, 100, 100, 100, 100, 100, 200, 200]

p.connect(p.GUI)
p.resetSimulation()
p.setTimeStep(stepsize)
p.setRealTimeSimulation(realtime)
p.setGravity(0, 0, -9.81)

urdfRootPath=pybullet_data.getDataPath()
planeId = p.loadURDF(os.path.join(urdfRootPath, "plane.urdf"),useFixedBase=True)
pandaUid = p.loadURDF(os.path.join(urdfRootPath, "franka_panda/panda.urdf"),useFixedBase=True)
trayUid = p.loadURDF(os.path.join(urdfRootPath, "tray/traybox.urdf"),basePosition=[-0.7, 0.5, 0])

objUid = p.loadURDF(os.path.join(urdfRootPath, "random_urdfs/004/004.urdf"), basePosition=[0.33, 0.5, 0.2], baseOrientation=[0, 0, 0.7071068, 0.7071068])
p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55, -0.35, 0.2])

# Robot parameters
#dof = p.getNumJoints(pandaUid) - 2 # Virtual fixed joint between the flange and last link
#print(dof)
dof = 9

joints = []
q_min = []
q_max = []
target_pos = []
target_torque = []

for j in range(dof):
    joint_info = p.getJointInfo(pandaUid, j)
    joints.append(j)
    q_min.append(joint_info[8])
    q_max.append(joint_info[9])
    target_pos.append((q_min[j] + q_max[j])/2.0)
    target_torque.append(0.)

# reset (initialize)
t = 0.0
for j in range(dof):
    target_pos[j] = (q_min[j] + q_max[j])/2.0
    target_torque[j] = 0.
    p.resetJointState(pandaUid,j,targetValue=target_pos[j])

p.setJointMotorControlArray(bodyUniqueId=pandaUid,
                            jointIndices=joints,
                            controlMode=p.VELOCITY_CONTROL,
                            forces=[0. for i in range(dof)])

init_pos, init_ori = p.getLinkState(pandaUid, 8, computeForwardKinematics=True)[:2]
while True:
    while True:
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        obj_states = p.getBasePositionAndOrientation(objUid) # input : object id , output : 3개의 포지션 값, 4개의 회전 값
        obj_pos = list(obj_states[0])
        obj_ori = list(obj_states[1])
        lfinger_pos, lfinger_ori = p.getLinkState(pandaUid, 9, computeForwardKinematics=True)[:2] # input : object id , link index
        rfinger_pos, rfinger_ori = p.getLinkState(pandaUid, 10, computeForwardKinematics=True)[:2]
        hand_pos, hand_ori = p.getLinkState(pandaUid, 8, computeForwardKinematics=True)[:2]
        print("distance : ",distance3D(lfinger_pos, rfinger_pos))
        # 정기구학 : 조인트 변수에 대하여 로봇 말단부의 위치/자세를 결정하는 과정
        # 역기구학 : 원하는 말단부의 위치/자세에 대응하는 조인트 변수를 결정하는 과정
        #print("robot-obj", distance3D(lfinger_pos, obj_pos))
        #print("jaw", distance3D(lfinger_pos, rfinger_pos))
        if distance1D(hand_pos[2], obj_pos[2]) < 0.1 and distance2D(hand_pos, obj_pos) < 0.05: # 손과 물체 사이의 높이가 0.06 이하이고 x,y 평면상의 좌표가 0.03 미만이면(물체를 잡을 수 있는 위치에 손이 놓여져 있다면)
            print("State 0: move to close and graps")
            print("hand and object position : ",hand_pos,obj_pos)
            print("hand and object orientation : ",hand_ori, obj_ori)
            print("height between hand and obj : ", distance1D(hand_pos[2],obj_pos[2]))
            print("hand and object x , y 거리 : ", distance2D(hand_pos, obj_pos))
            print("hand and object 3차원 거리 : ",distance3D(hand_pos,obj_pos))
            goal_pos = obj_pos
            goal_ori = obj_ori
            goal_pos[2] = obj_pos[2] # 물체의 z축 높이와 같도록 손의 위치를 조정한다.
            target_pos = list(p.calculateInverseKinematics(pandaUid, 8, goal_pos, [1, 0, goal_ori[2], 0])) # input : object id , end effector link index(말단 장치) , targetPosition , targetOrientation
            p.setJointMotorControlArray(bodyUniqueId=pandaUid,
                                        jointIndices=joints,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPositions=target_pos,
                                        forces=max_torque,
                                        positionGains=position_control_gain_p,
                                        velocityGains=position_control_gain_d)
            p.setJointMotorControl2(pandaUid, 9, p.POSITION_CONTROL, 0.0, force=200) # input : object id , jointindex , controlmode , targetposition , force
            p.setJointMotorControl2(pandaUid, 10, p.POSITION_CONTROL, 0.0, force=200)
            print(distance3D(hand_pos,goal_pos))
            if distance3D(hand_pos, obj_pos) < 0.1: # 손이 물체를 잡았을때 손과 물체 사이의 거리가 0.07 미만인 경우 다음 단계로 넘어간다.
                break

        else:
            goal_pos = obj_pos
            goal_ori = obj_ori

            if hand_pos[2] > goal_pos[2] and distance2D(hand_pos,goal_pos) < 0.04: # 손의 높이가 물체의 높이보다 높고 x,y 평면 상의 거리가 0.03 미만이면(손의 위치가 물체 바로 위에 +z축 방향으로 떨어져있다면 손을 내려서 물체를 잡아야한다.)
                print("state 2: hand down")
                print("hand and object position : ", hand_pos, obj_pos)
                print("hand and object orientation : ", hand_ori, obj_ori)
                print("height between hand and obj : ", distance1D(hand_pos[2], obj_pos[2]))
                print("hand and object x , y 거리 : ", distance2D(hand_pos, goal_pos))
                goal_pos[2] = obj_pos[2] # 손의 높이를 물체의 높이와 같게한다.

                target_pos = list(p.calculateInverseKinematics(pandaUid, 8, goal_pos, [9, goal_ori[1], goal_ori[2], goal_ori[3]]))
                p.setJointMotorControlArray(bodyUniqueId=pandaUid,
                                            jointIndices=joints,
                                            controlMode=p.POSITION_CONTROL,
                                            targetPositions=target_pos,
                                            forces=max_torque,
                                            positionGains=position_control_gain_p,
                                            velocityGains=position_control_gain_d)
                p.setJointMotorControl2(pandaUid, 9, p.POSITION_CONTROL, 0.08)
                p.setJointMotorControl2(pandaUid, 10, p.POSITION_CONTROL, 0.08)
                t += stepsize
                p.stepSimulation()
                time.sleep(stepsize)

            else: # 그 외 손의 위치가 물체와 멀리 떨어져 있는 경우
                print("state 1: move to close object")
                print("hand and object position : ", hand_pos, obj_pos)
                print("hand and object orientation : ", hand_ori, obj_ori)
                print("hand and object x , y 거리 : ", distance2D(hand_pos, goal_pos))
                print("height between hand and obj : ", distance1D(hand_pos[2], obj_pos[2]))
                goal_pos[2] = 0.4 # 물체를 위에서 아래로 pick 해야하기 때문에 손의 위치를 +z축 방향으로 0.4로 유지한다.

                target_pos = list(p.calculateInverseKinematics(pandaUid, 8, goal_pos, [9, goal_ori[1], goal_ori[2], goal_ori[3]]))

                p.setJointMotorControlArray(bodyUniqueId=pandaUid,
                                            jointIndices=joints,
                                            controlMode=p.POSITION_CONTROL,
                                            targetPositions=target_pos,
                                            forces=max_torque,
                                            positionGains=position_control_gain_p,
                                            velocityGains=position_control_gain_d)
                p.setJointMotorControl2(pandaUid, 9, p.POSITION_CONTROL, 0.08)
                p.setJointMotorControl2(pandaUid, 10, p.POSITION_CONTROL, 0.08)

                t += stepsize
                p.stepSimulation()
                time.sleep(stepsize)

    t = 0.0
    goal_pos = [-0.7, 0.5, 0.2]
    while True: # 물체를 pick 한 상태로 목표지점까지 물체를 운반한다.
        print("State 4: move to target position")
        print("hand move to target")
        print("hand and goal position : ", hand_pos, goal_pos)
        print("hand and goal orientation : ", hand_ori, goal_ori)
        print("hand and goal x , y 거리 : ", distance2D(hand_pos, goal_pos))
        print("height between hand and obj : ", distance1D(hand_pos[2], obj_pos[2]))

        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        hand_pos, hand_ori = p.getLinkState(pandaUid, 8, computeForwardKinematics=True)[:2]
        lfinger_pos, lfinger_ori = p.getLinkState(pandaUid, 9, computeForwardKinematics=True)[:2]
        goal_pos[2]=0.4 # +z축 방향으로 0.4 만큼의 높이를 유지한채로 물체를 운반한다.

        target_pos = list(p.calculateInverseKinematics(pandaUid, 8, goal_pos, [1, 0, 0, 0]))
        p.setJointMotorControlArray(bodyUniqueId=pandaUid,
                                    jointIndices=joints,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=target_pos,
                                    forces=max_torque,
                                    positionGains=position_control_gain_p,
                                    velocityGains=position_control_gain_d)
        t += stepsize
        p.stepSimulation()
        time.sleep(stepsize)

        #print(lfinger_pos)
        if distance2D(hand_pos, goal_pos) < 0.075: # 손의 위치와 목표 지점과의 x,y 평면 상의 거리가 0.075 미만일 때 물체가 목표 지점에 도착했다고 판단한다.
            print("target pose Reached")
            break

    if distance2D(obj_pos, goal_pos) < 0.1:  # 물체의 위치와 목표 지점과의 x,y 평면 상의 거리가 0.1 미만일 때 물체가 목표 지점에 도착했다고 판단한다.
        print("final Reached")
        break

print("State 5: end move")
while True: # 목표지점에 도착한 후 물체를 놓는다.
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
    p.setJointMotorControl2(pandaUid, 9, p.POSITION_CONTROL, 0.08)
    p.setJointMotorControl2(pandaUid, 10, p.POSITION_CONTROL, 0.08)
    t += stepsize
    p.stepSimulation()
    time.sleep(stepsize)