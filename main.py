from panda_env import PandaEnv

def process_result(process) : # prints process, returns end condition
    if process == 0:
        print("--Grasped a duck--")
        return False, process + 1
    if process == 1:
        print("--Moving to the box--")
        return False, process + 1
    if process == 2:
        print("--release the duck--")
        return False, process + 1
    if process == 3:
        print("--init panda's pos--")
        return False, process + 1
    if process == 4:
        return True, 0
    else :
        return False, process

def main() :
    env = PandaEnv()
    done = False
    error = 0.01

    k_p = 3
    k_d = 1
    dt = 1. / 240.  # the default timestep in pybullet is 240 Hz

    observation = env.reset()

    for i in range(1, 4) :
        isEnd = False
        process = 0
        fingers = 1
        object_position = env.duck_position(i)
        # env.resetJoints_panda()
        print('object_position', object_position)

        for t in range(300):
            env.render()
            dx = object_position[0] - observation[0]
            dy = object_position[1] - observation[1]
            target_z = object_position[2] - 0.007

            if (observation[3] + observation[4]) < error + 0.03 and fingers == 0:
                target_z = 0.5
            if process == 2 : # put a duck in the box
                target_z = 0
            elif process == 3 : # release a duck
                fingers = 1
            elif process == 4 : # init panda's pos
                target_z = 0.5
                dx = 0.3 - observation[0]
            dz = target_z - observation[2]

            if abs(dx) < error and abs(dy) < error and abs(dz) < error:
                fingers = 0     # to grasp

            pd_x = k_p * dx + k_d * dx / dt
            pd_y = k_p * dy + k_d * dy / dt
            pd_z = k_p * dz + k_d * dz / dt
            action = [pd_x, pd_y, pd_z, fingers]
            print("action : ", action)
            observation, reward, done, info = env.step(action, process, i-1)
            object_position = info['object_position']

            if done:
                isEnd, process = process_result(process)
                if isEnd :
                    break
                else :
                    continue
    env.close()


if __name__ == "__main__":
    main()
