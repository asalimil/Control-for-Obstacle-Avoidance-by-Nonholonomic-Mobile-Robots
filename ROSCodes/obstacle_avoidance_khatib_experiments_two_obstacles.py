#! /usr/bin/env python
import rospy
import numpy
import math
from numpy import linalg as LA
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan



def myAtan2(xr,yr):
    if xr > 0 and yr >= 0:
        alpha = numpy.arctan(yr/xr)
    elif xr < 0 and yr >= 0:
        alpha = numpy.arctan(yr/xr) + math.pi
    elif xr < 0 and yr < 0:
        alpha = numpy.arctan(yr/xr) + math.pi
    elif xr > 0 and yr < 0:
        alpha = numpy.arctan(yr/xr)
    elif xr == 0 and yr > 0:
        alpha = math.pi/2
    elif xr == 0 and yr <= 0:
        alpha = 3*math.pi/2
    return alpha


def callback_odom(odom):
    global X
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    qz = odom.pose.pose.orientation.z
    qw = odom.pose.pose.orientation.w
    X = numpy.array([x, y, qz, qw])



# node initialization
rospy.init_node('move_node')



# initial point
x0 = -1.0; y0 = 2.0; th0 = 0.0*math.pi/2
# alpha = 0.0

X0 = numpy.array([x0, y0, th0])
X = numpy.array([0.0,0.0, 0.0, 0.0])
# X = X0
ne = numpy.Inf


# robot controller
while ne > 0.05:

    # subscriber odometry
    sub1 = rospy.Subscriber('/tb3_1/odom', Odometry, callback_odom)
    # publisher velocity
    pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
    move = Twist()
    #
    xr = X[0] + X0[0]
    yr = X[1] + X0[1]
    qz = X[2]
    qw = X[3]
    Xr = numpy.array([xr, yr])
    #
    # k = 5.0
    # k = 3.0
    #
    # d_star = 0.5; eta = 0.5 
    d_star = 0.5; eta = 0.6
    d = LA.norm(Xr[0:2])
    if d <= d_star:
        grad_U_att = eta*Xr
    else:
        grad_U_att = d_star*eta*Xr/d
    

    #
    # O1 = numpy.array([-0.6, 3.3])
    # O2 = numpy.array([0.6, 3.3])
    # O3 = numpy.array([0.0, 2.3])
    # O4 = numpy.array([1.0, 2.3])
    # O5 = numpy.array([-0.6, 1.3])
    # O6 = numpy.array([0.6, 1.3])
    O1 = numpy.array([0.0, 1.0])
    O2 = numpy.array([-1.0, 0.5])

    #
    r1 = 0.2; r2 = 0.125
    #
    D1 = numpy.sqrt(numpy.power(LA.norm(Xr[0:2]-O1), 2) - r1*r1); D1_dot = 0.5*numpy.power(numpy.power(LA.norm(Xr[0:2]-O1),2)-r1*r1,-0.5)*2*(Xr[0:2]-O1)
    D2 = numpy.sqrt(numpy.power(LA.norm(Xr[0:2]-O2), 2) - r2*r2); D2_dot = 0.5*numpy.power(numpy.power(LA.norm(Xr[0:2]-O2),2)-r2*r2,-0.5)*2*(Xr[0:2]-O2)

    #
    # Q_star = 0.5; zeta = 0.25 
    # Q_star = 0.3; zeta = 0.15 
    Q_star = 0.6; zeta = 0.15 
    #   
    if D1 <= Q_star:
        grad_U_rep_1 = zeta*((1/Q_star) - (1/D1))*(1/numpy.power(D1,2))*D1_dot
    else:
        grad_U_rep_1 = 0.0

    #
    if D2 <= Q_star:
        grad_U_rep_2 = zeta*((1/Q_star) - (1/D2))*(1/numpy.power(D2,2))*D2_dot
    else:
        grad_U_rep_2 = 0.0

 

    grad_U_rep = grad_U_rep_1 + grad_U_rep_2
    grad_phi = grad_U_att + grad_U_rep
    #
    # Obstacle Avoidance
    p = numpy.sqrt(numpy.power(grad_phi[0],2) + numpy.power(grad_phi[1],2))
    alpha = myAtan2(grad_phi[0],grad_phi[1])
    q = (2*qz*qw*numpy.cos(alpha)) - ((numpy.power(qw,2)-numpy.power(qz,2))*numpy.sin(alpha))
    #
    # kp = 0.05; kq = 0.15
    # kp = 0.1; kq = 0.25
    # kp = -0.1; kq = -0.25
    # kp = 0.5; kq = 2.5
    kp = 0.25; kq = 0.5
    # control law
    u = kp*p
    w = kq*q
    #
    # print("Xr: ", Xr)
    print("U: ", u, w)
    # print(numpy.round(Xr[0],4), numpy.round(Xr[1]),4)
    # control command
    move.linear.x = u
    move.angular.z = w
    # move robot
    pub.publish(move)
    rate = rospy.Rate(2)
    ne = LA.norm(Xr[0:2])




# Stop robot
pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=1)
move = Twist()
move.linear.x = 0.0; move.angular.z = 0.0
pub.publish(move)
print("Robot has arrived to GOAL point!")
rospy.spin()
