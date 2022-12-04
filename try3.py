#!/usr/bin/python3


import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

PI = 3.1415926535897
regions = {
    'bright': 1,
    'fright': 1,
    'front':  1,
    'fleft':  1,
    'bleft':  1,
}


def laser_callback(msg):
    global regions
    regions = {
        'bright': min(min(msg.ranges[0:143]), 8),
        'fright':  min(min(msg.ranges[144:287]), 8),
        'front':   min(min(msg.ranges[288:431]), 8),
        'fleft':   min(min(msg.ranges[432:575]), 8),
        'bleft':   min(min(msg.ranges[576:713]), 8),
    }
    rospy.loginfo(regions)


def control_loop():
    rospy.init_node('ebot_controller')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
    start_flag = 0
    angle_travled = 0
    left_count = 0
    side_flag = 0
    previous_time = rospy.Time.now()
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        duration = current_time - previous_time
        linear_x = 1
        angular_z = 0
        state_description = ''

        if start_flag == 1:
            if angle_travled >= 6:
                side_flag = 1

            if side_flag == 0:
                linear_x, angular_z, state_description = control_state(
                    'fleft', 'bleft', 0.5, 1.2, 1)
            elif left_count > 20:
                linear_x, angular_z, state_description = control_state(
                    'fleft', 'bleft', 0.5, 0.7, 0.7)
            else:
                linear_x, angular_z, state_description = control_state(
                    'fright', 'bright', 0.5, 1.2, 1)
                if regions['bleft'] < 1:
                    left_count += 1

        else:
            if regions['bleft'] < 1:
                start_flag = 1

        rospy.loginfo(state_description + '  angle = ' +
                      str(angle_travled) + ' left_count = '+str(left_count))
        if state_description.find('1') != -1:
            angle_travled += calculate_angle(angular_z, duration)
        velocity_msg.linear.x = linear_x
        velocity_msg.angular.z = angular_z
        pub.publish(velocity_msg)
        rate.sleep()
        previous_time = current_time


def check_lane(bside):
    b_side = regions[bside]
    if b_side > 0.4 and b_side < 0.6:
        return 0
    elif b_side > 0.65:
        return 0.3
    else:
        return -0.3


def calculate_angle(rad_speed, duration):
    return rad_speed * duration.to_sec()


def control_state(fside, bside, range_f, range_fside, range_bside, other_side='bright'):
    # standard condition
    linear_x = 1
    angular_z = 0
    state_description = ''

    # which side to focus on
    if bside == 'bleft':
        sign = 1
    else:
        other_side = 'bleft'
        sign = -1

    # if very close to hit something
    if regions['front'] < 0.7 and regions[fside] < 0.5 and regions[bside] < 0.4:
        state_description = 'warning -  back and turn other side'
        linear_x = -0.5
        angular_z = -1 * sign
        return (linear_x, angular_z, state_description)
    elif regions[other_side] < 0.3:
        state_description = 'warning -  back and turn other side'
        linear_x = 0.1
        angular_z = 1 * sign
        return (linear_x, angular_z, state_description)

    # the cases and what to do at each one
    if regions['front'] > range_f and regions[fside] > range_fside and regions[bside] > range_bside:
        state_description = 'case 1 -  turn '+bside
        linear_x = 0.1  # return check this
        angular_z = 2 * sign
    elif regions['front'] < range_f and regions[fside] > range_fside and regions[bside] > range_bside:
        state_description = 'case 2 - stop and turn '+bside
        linear_x = 0
        angular_z = 0.5 * sign
    elif regions['front'] > range_f and regions[fside] < range_fside and regions[bside] > range_bside:
        state_description = 'case 3 - v1 turn '+bside
        linear_x = 0.1
        angular_z = -0.5 * sign
    elif regions['front'] > range_f and regions[fside] > range_fside and regions[bside] < range_bside:
        state_description = 'case 4 - slow reorient to middle'
        linear_x = 0.5
        angular_z = check_lane(bside)
    elif regions['front'] < range_f and regions[fside] < range_fside and regions[bside] > range_bside:
        state_description = 'case 5 - obstacle on front and '+fside
        linear_x = 0
        angular_z = 0.5 * sign
    elif regions['front'] > range_f and regions[fside] < range_fside and regions[bside] < range_bside:
        state_description = 'case 6 - reorient to middle'
        linear_x = 0.7
        angular_z = check_lane(bside)
    elif regions['front'] < range_f and regions[fside] > range_fside and regions[bside] < range_bside:
        state_description = 'case 7 - reorient'
        linear_x = 0
        angular_z = 0.1 * sign
    elif regions['front'] < range_f and regions[fside] < range_fside and regions[bside] < range_bside:
        state_description = 'case 8 - stop and face other side'
        linear_x = -0.2
        angular_z = -2 * sign
    else:
        state_description = 'Unknown state'
        linear_x = 0
        angular_z = 0

    return (linear_x, angular_z, state_description)


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
