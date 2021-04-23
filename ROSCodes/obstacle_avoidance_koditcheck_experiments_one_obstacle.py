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
x0 = 0.0; y0 = 2.5; th0 = 0.0*math.pi/2
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
    # global Xr
    xr = X[0] + X0[0]
    yr = X[1] + X0[1]
    qz = X[2]
    qw = X[3]
    Xr = numpy.array([xr, yr])
    #
    # k = 5.0
    k = 3.0
    #
    Oarena = numpy.array([0.0, 1.5])
    O1 = numpy.array([-0.5, 1.5])
    #
    rarena = 2.0; r1 = 0.15
    #
    darena = LA.norm(Xr[0:2]-Oarena)
    d1 = LA.norm(Xr[0:2]-O1);
    #
    beta_arena = -darena*darena + rarena*rarena; betadot_arena = -2*(Xr[0:2]-Oarena)
    beta_1 = d1*d1 - r1*r1; betadot_1 = 2*(Xr[0:2]-O1)
    # beta
    beta = beta_arena*beta_1
    # betadot
    betadot =  betadot_arena*beta_1 + beta_arena*betadot_1
    #
    num_phi = numpy.power(LA.norm(Xr[0:2]),2)
    den_phi = numpy.power(beta + numpy.power((num_phi),k), 1/k)
    # phi = num_phi/den_phi
    #
    grad_num  = 2*Xr
    grad_den = (1/k)*numpy.power(beta + numpy.power((num_phi),k),-1+(1/k))*(betadot + k*numpy.power(LA.norm(Xr[0:2]),2*(k-1))*2*Xr[0:2])
    num_grad_phi = grad_num*den_phi - grad_den*num_phi
    den_grad_phi = numpy.power(den_phi,2)
    grad_phi = num_grad_phi/den_grad_phi
    # Obstacle Avoidance
    p = numpy.sqrt(numpy.power(grad_phi[0],2) + numpy.power(grad_phi[1],2))
    alpha = myAtan2(grad_phi[0],grad_phi[1])
    q = (2*qz*qw*numpy.cos(alpha)) - ((numpy.power(qw,2)-numpy.power(qz,2))*numpy.sin(alpha))
    #
    pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
    move = Twist()
    # kp = -0.1; kq = -0.5
    kp = 1.5; kq = 2.5
    # kp = -1.5; kq = -2.5
    # control law
    u = kp*p
    w = kq*q
    #
    print("Xr: ", Xr)
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
