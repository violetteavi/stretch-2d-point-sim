import stretch_body.robot
import signal
import time
import math
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

XR_MIN = 0.15
XR_MAX = 0.45
YR_MIN = 0.56
YR_MAX = 0.86
XS_MIN = -2
XS_MAX = 2
YS_MIN = -2
YS_MAX = 10

def real2sim_coords(xr, yr):
    xs = XS_MIN + (XS_MAX-XS_MIN)*(xr-XR_MIN)/(XR_MAX-XR_MIN)
    ys = YS_MIN + (YS_MAX-YS_MIN)*(yr-YR_MIN)/(YR_MAX-YR_MIN)
    return xs, ys
    
def sim2real_coords(xs, ys):
    xr = XR_MIN + (XR_MAX-XR_MIN)*(xs-XS_MIN)/(XS_MAX-XS_MIN)
    yr = YR_MIN + (YR_MAX-YR_MIN)*(ys-YS_MIN)/(YS_MAX-YS_MIN)
    return xr, yr

def sim2real_vel(vxs, vys):
    vxr = vxs*(XR_MAX-XR_MIN)/(XS_MAX-XS_MIN)
    vyr = vys*(YR_MAX-YR_MIN)/(YS_MAX-YS_MIN)
    return vxr, vyr
    
def real2sim_vel(vxr, vyr):
    vxs = vxr*(XS_MAX-XS_MIN)/(XR_MAX-XR_MIN)
    vys = vyr*(YS_MAX-YS_MIN)/(YR_MAX-YR_MIN)
    return vxs, vys
    
def roundtrip_test():
    xr = 0.3
    yr = 0.75
    xs, ys = real2sim_coords(xr, yr)
    xr_new, yr_new = sim2real_coords(xs, ys)
    if(not math.isclose(xr, xr_new) or not math.isclose(yr, yr_new)):
        print("Something is wrong with coord round trip")
    vxs = 1
    vys = 2
    vxr, vyr = sim2real_vel(vxs, vys)
    vxs_new, vys_new = real2sim_vel(vxr, vyr)
    if(not math.isclose(vxs, vxs_new) or not math.isclose(vys, vys_new)):
        print("Something is wrong with vel round trip")
    

if __name__ == "__main__":
    stop_robot()
    roundtrip_test()
    #reset()
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

