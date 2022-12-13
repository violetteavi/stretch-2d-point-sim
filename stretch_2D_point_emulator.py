import stretch_body.robot
import signal
import time
robot = stretch_body.robot.Robot()
robot.startup()

def stop_robot():
    #robot.end_of_arm.move_to('wrist_yaw', 0)
    #robot.end_of_arm.move_to('stretch_gripper', 60)
    robot.lift.set_velocity(v_m=0)
    robot.arm.set_velocity(v_m=0)
    robot.push_command()

def clean_shutdown(signum, frame):
    stop_robot()
    exit(1)

signal.signal(signal.SIGINT, clean_shutdown)

def print_arm_status():
    print("=======================")
    print("robot.arm.status['pos']")
    print(robot.arm.status['pos'])
    print("robot.lift.status['pos']")
    print(robot.lift.status['pos'])
    print("robot.end_of_arm.status['stretch_gripper']['pos_pct']")
    print(robot.end_of_arm.status['stretch_gripper']['pos_pct'])

def send_velocity(vx, vy):
    robot.lift.set_velocity(v_m=vy)
    robot.arm.set_velocity(v_m=vx)
    robot.push_command()

def get_position():
    px = robot.arm.status['pos']
    py = robot.lift.status['pos']
    return px, py

def compute_velocity_swirl(px, py):
    cx = 0.25
    cy = 0.5
    speed_multiplier = 1 # m/s/m
    swirl_coefficient = 0.9 # amount to swirl vs seek center. [0,1]
    rx = px - cx
    ry = py - cy
    vx_center = -rx * speed_multiplier
    vy_center = -ry * speed_multiplier
    vx_swirl = -ry * speed_multiplier
    vy_swirl = rx * speed_multiplier
    vx = (1-swirl_coefficient)*vx_center + swirl_coefficient*vx_swirl
    vy = (1-swirl_coefficient)*vy_center + swirl_coefficient*vy_swirl
    return vx, vy

def move_to_blocking(x, y):
    robot.arm.move_to(x)
    robot.lift.move_to(y)
    robot.push_command()
    time.sleep(5)

def drop_keys():
    #move_to_blocking(0.3, 0.56)
    move_to_blocking(0.15, 0.56)
    move_to_blocking(0.15, 0.86)
    move_to_blocking(0.3, 0.86)
    robot.end_of_arm.move_to('stretch_gripper', 60)
    robot.push_command()
    time.sleep(5)

def reset():
    #move_to_blocking(0.3, 0.86)
    move_to_blocking(0.15, 0.86)
    move_to_blocking(0.15, 0.56)
    move_to_blocking(0.3, 0.56)
    robot.end_of_arm.move_to('stretch_gripper', 0)
    robot.push_command()
    time.sleep(5)

if __name__ == "__main__":
    stop_robot()
    reset()
    #drop_keys()
    # update new position
    robot.push_command()
    loop_count = 0
    while False:
        px, py = get_position()
        vx, vy = compute_velocity_swirl(px, py)
        send_velocity(vx, vy)
        if(loop_count%10==0):
            print_arm_status()
        loop_count = loop_count + 1
        time.sleep(0.1)

