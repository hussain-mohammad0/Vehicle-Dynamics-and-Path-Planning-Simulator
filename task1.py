#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from math import cos, sin, tan



L = 0.324 

def state_transition_dynamics(state, alpha, delta, dt):
    X, Y, theta, v = state
    v_new = max(min(v + alpha * dt, 5), 0) 
    theta_new = theta + (v_new / L) * dt * tan(delta)
    X_new = X + v_new * cos(theta_new) * dt
    Y_new = Y + v_new * sin(theta_new) * dt
    return [X_new, Y_new, theta_new, v_new]

def kinematic_update_publisher():
    rospy.init_node('pa1_task1_node', anonymous=True)
    pub = rospy.Publisher('/kinematic_update', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(20)  
    control_list = rospy.get_param('/control_list')
    state = [0, 0, 0, 0]  

    for control in control_list:
        alpha, delta, duration = control
        for _ in range(int(duration * 10)): 
            state = state_transition_dynamics(state, alpha, delta, 0.1)
            msg = Float32MultiArray(data=state)
            pub.publish(msg)
            rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    try:
        kinematic_update_publisher()
    except rospy.ROSInterruptException:
        pass