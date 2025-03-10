from sympy import sin, cos, zeros, eye, symbols, sqrt, lambdify, Matrix
import numpy as np


class CentralisedJacobian():
    yaw, pitch, roll, q_1, q_2, q_3 = symbols('yaw pitch roll q_1 q_2 q_3')

    # This geometric jacobian was derived in the am_kinematics repository with sympy.
    jacobian = zeros(6, 9)
    jacobian[0,0]=1.00000000000000
    jacobian[0,3]=-0.273*sin(pitch)*sin(q_3)*sin(yaw)*sin(q_1 + roll) + 0.273*sin(pitch)*sin(yaw)*cos(q_2)*cos(q_3)*cos(q_1 + roll) + 0.311*sin(pitch)*sin(yaw)*cos(q_2)*cos(q_1 + roll) + 0.11*sin(pitch)*sin(yaw)*cos(q_1 + roll) + 0.273*sin(q_2)*sin(yaw)*cos(pitch)*cos(q_3) + 0.311*sin(q_2)*sin(yaw)*cos(pitch) - 0.273*sin(q_3)*cos(yaw)*cos(q_1 + roll) - 0.273*sin(q_1 + roll)*cos(q_2)*cos(q_3)*cos(yaw) - 0.311*sin(q_1 + roll)*cos(q_2)*cos(yaw) - 0.11*sin(q_1 + roll)*cos(yaw)
    jacobian[0,4]=0.273*sin(pitch)*sin(q_2)*cos(q_3)*cos(yaw) + 0.311*sin(pitch)*sin(q_2)*cos(yaw) + 0.273*sin(q_3)*sin(q_1 + roll)*cos(pitch)*cos(yaw) - 0.273*cos(pitch)*cos(q_2)*cos(q_3)*cos(yaw)*cos(q_1 + roll) - 0.311*cos(pitch)*cos(q_2)*cos(yaw)*cos(q_1 + roll) - 0.11*cos(pitch)*cos(yaw)*cos(q_1 + roll)
    jacobian[0,5]=0.273*sin(pitch)*sin(q_3)*cos(yaw)*cos(q_1 + roll) + 0.273*sin(pitch)*sin(q_1 + roll)*cos(q_2)*cos(q_3)*cos(yaw) + 0.311*sin(pitch)*sin(q_1 + roll)*cos(q_2)*cos(yaw) + 0.11*sin(pitch)*sin(q_1 + roll)*cos(yaw) + 0.273*sin(q_3)*sin(yaw)*sin(q_1 + roll) - 0.273*sin(yaw)*cos(q_2)*cos(q_3)*cos(q_1 + roll) - 0.311*sin(yaw)*cos(q_2)*cos(q_1 + roll) - 0.11*sin(yaw)*cos(q_1 + roll)
    jacobian[0,6]=0.273*sin(pitch)*sin(q_3)*cos(yaw)*cos(q_1 + roll) + 0.273*sin(pitch)*sin(q_1 + roll)*cos(q_2)*cos(q_3)*cos(yaw) + 0.311*sin(pitch)*sin(q_1 + roll)*cos(q_2)*cos(yaw) + 0.11*sin(pitch)*sin(q_1 + roll)*cos(yaw) + 0.273*sin(q_3)*sin(yaw)*sin(q_1 + roll) - 0.273*sin(yaw)*cos(q_2)*cos(q_3)*cos(q_1 + roll) - 0.311*sin(yaw)*cos(q_2)*cos(q_1 + roll) - 0.11*sin(yaw)*cos(q_1 + roll)
    jacobian[0,7]=0.273*sin(pitch)*sin(q_2)*cos(q_3)*cos(yaw)*cos(q_1 + roll) + 0.311*sin(pitch)*sin(q_2)*cos(yaw)*cos(q_1 + roll) + 0.273*sin(q_2)*sin(yaw)*sin(q_1 + roll)*cos(q_3) + 0.311*sin(q_2)*sin(yaw)*sin(q_1 + roll) - 0.273*cos(pitch)*cos(q_2)*cos(q_3)*cos(yaw) - 0.311*cos(pitch)*cos(q_2)*cos(yaw)
    jacobian[0,8]=0.273*sin(pitch)*sin(q_3)*cos(q_2)*cos(yaw)*cos(q_1 + roll) + 0.273*sin(pitch)*sin(q_1 + roll)*cos(q_3)*cos(yaw) + 0.273*sin(q_2)*sin(q_3)*cos(pitch)*cos(yaw) + 0.273*sin(q_3)*sin(yaw)*sin(q_1 + roll)*cos(q_2) - 0.273*sin(yaw)*cos(q_3)*cos(q_1 + roll)
    jacobian[1,1]=1.00000000000000
    jacobian[1,3]=0.273*sin(pitch)*sin(q_3)*sin(q_1 + roll)*cos(yaw) - 0.273*sin(pitch)*cos(q_2)*cos(q_3)*cos(yaw)*cos(q_1 + roll) - 0.311*sin(pitch)*cos(q_2)*cos(yaw)*cos(q_1 + roll) - 0.11*sin(pitch)*cos(yaw)*cos(q_1 + roll) - 0.273*sin(q_2)*cos(pitch)*cos(q_3)*cos(yaw) - 0.311*sin(q_2)*cos(pitch)*cos(yaw) - 0.273*sin(q_3)*sin(yaw)*cos(q_1 + roll) - 0.273*sin(yaw)*sin(q_1 + roll)*cos(q_2)*cos(q_3) - 0.311*sin(yaw)*sin(q_1 + roll)*cos(q_2) - 0.11*sin(yaw)*sin(q_1 + roll)
    jacobian[1,4]=0.273*sin(pitch)*sin(q_2)*sin(yaw)*cos(q_3) + 0.311*sin(pitch)*sin(q_2)*sin(yaw) + 0.273*sin(q_3)*sin(yaw)*sin(q_1 + roll)*cos(pitch) - 0.273*sin(yaw)*cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll) - 0.311*sin(yaw)*cos(pitch)*cos(q_2)*cos(q_1 + roll) - 0.11*sin(yaw)*cos(pitch)*cos(q_1 + roll)
    jacobian[1,5]=0.273*sin(pitch)*sin(q_3)*sin(yaw)*cos(q_1 + roll) + 0.273*sin(pitch)*sin(yaw)*sin(q_1 + roll)*cos(q_2)*cos(q_3) + 0.311*sin(pitch)*sin(yaw)*sin(q_1 + roll)*cos(q_2) + 0.11*sin(pitch)*sin(yaw)*sin(q_1 + roll) - 0.273*sin(q_3)*sin(q_1 + roll)*cos(yaw) + 0.273*cos(q_2)*cos(q_3)*cos(yaw)*cos(q_1 + roll) + 0.311*cos(q_2)*cos(yaw)*cos(q_1 + roll) + 0.11*cos(yaw)*cos(q_1 + roll)
    jacobian[1,6]=0.273*sin(pitch)*sin(q_3)*sin(yaw)*cos(q_1 + roll) + 0.273*sin(pitch)*sin(yaw)*sin(q_1 + roll)*cos(q_2)*cos(q_3) + 0.311*sin(pitch)*sin(yaw)*sin(q_1 + roll)*cos(q_2) + 0.11*sin(pitch)*sin(yaw)*sin(q_1 + roll) - 0.273*sin(q_3)*sin(q_1 + roll)*cos(yaw) + 0.273*cos(q_2)*cos(q_3)*cos(yaw)*cos(q_1 + roll) + 0.311*cos(q_2)*cos(yaw)*cos(q_1 + roll) + 0.11*cos(yaw)*cos(q_1 + roll)
    jacobian[1,7]=0.273*sin(pitch)*sin(q_2)*sin(yaw)*cos(q_3)*cos(q_1 + roll) + 0.311*sin(pitch)*sin(q_2)*sin(yaw)*cos(q_1 + roll) - 0.273*sin(q_2)*sin(q_1 + roll)*cos(q_3)*cos(yaw) - 0.311*sin(q_2)*sin(q_1 + roll)*cos(yaw) - 0.273*sin(yaw)*cos(pitch)*cos(q_2)*cos(q_3) - 0.311*sin(yaw)*cos(pitch)*cos(q_2)
    jacobian[1,8]=0.273*sin(pitch)*sin(q_3)*sin(yaw)*cos(q_2)*cos(q_1 + roll) + 0.273*sin(pitch)*sin(yaw)*sin(q_1 + roll)*cos(q_3) + 0.273*sin(q_2)*sin(q_3)*sin(yaw)*cos(pitch) - 0.273*sin(q_3)*sin(q_1 + roll)*cos(q_2)*cos(yaw) + 0.273*cos(q_3)*cos(yaw)*cos(q_1 + roll)
    jacobian[2,2]=1.00000000000000
    jacobian[2,4]=-0.273*sin(pitch)*sin(q_3)*sin(q_1 + roll) + 0.273*sin(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll) + 0.311*sin(pitch)*cos(q_2)*cos(q_1 + roll) + 0.11*sin(pitch)*cos(q_1 + roll) + 0.273*sin(q_2)*cos(pitch)*cos(q_3) + 0.311*sin(q_2)*cos(pitch)
    jacobian[2,5]=0.273*sin(q_3)*cos(pitch)*cos(q_1 + roll) + 0.273*sin(q_1 + roll)*cos(pitch)*cos(q_2)*cos(q_3) + 0.311*sin(q_1 + roll)*cos(pitch)*cos(q_2) + 0.11*sin(q_1 + roll)*cos(pitch)
    jacobian[2,6]=0.273*sin(q_3)*cos(pitch)*cos(q_1 + roll) + 0.273*sin(q_1 + roll)*cos(pitch)*cos(q_2)*cos(q_3) + 0.311*sin(q_1 + roll)*cos(pitch)*cos(q_2) + 0.11*sin(q_1 + roll)*cos(pitch)
    jacobian[2,7]=0.273*sin(pitch)*cos(q_2)*cos(q_3) + 0.311*sin(pitch)*cos(q_2) + 0.273*sin(q_2)*cos(pitch)*cos(q_3)*cos(q_1 + roll) + 0.311*sin(q_2)*cos(pitch)*cos(q_1 + roll)
    jacobian[2,8]=-0.273*sin(pitch)*sin(q_2)*sin(q_3) + 0.273*sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + 0.273*sin(q_1 + roll)*cos(pitch)*cos(q_3)
    jacobian[3,4]=((sin(pitch)*sin(q_2)*sin(q_3) - sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) - sin(q_1 + roll)*cos(pitch)*cos(q_3))*(-sin(pitch)*sin(q_3)*sin(q_1 + roll) + sin(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll) + sin(q_2)*cos(pitch)*cos(q_3))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2) + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))*(-sin(pitch)*sin(q_3)*cos(q_2)*cos(q_1 + roll) - sin(pitch)*sin(q_1 + roll)*cos(q_3) - sin(q_2)*sin(q_3)*cos(pitch))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2))*cos(pitch)*cos(yaw) + (-sin(pitch)*sin(q_2)*cos(q_1 + roll) + cos(pitch)*cos(q_2))*sin(yaw)/sqrt(1 - (sin(pitch)*cos(q_2) + sin(q_2)*cos(pitch)*cos(q_1 + roll))**2)
    jacobian[3,5]=((sin(q_3)*cos(pitch)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_2)*cos(q_3))*(sin(pitch)*sin(q_2)*sin(q_3) - sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) - sin(q_1 + roll)*cos(pitch)*cos(q_3))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2) + (-sin(q_3)*sin(q_1 + roll)*cos(pitch)*cos(q_2) + cos(pitch)*cos(q_3)*cos(q_1 + roll))*(sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2))*cos(pitch)*cos(yaw) - sin(q_2)*sin(yaw)*sin(q_1 + roll)*cos(pitch)/sqrt(1 - (sin(pitch)*cos(q_2) + sin(q_2)*cos(pitch)*cos(q_1 + roll))**2)
    jacobian[3,6]=((sin(q_3)*cos(pitch)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_2)*cos(q_3))*(sin(pitch)*sin(q_2)*sin(q_3) - sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) - sin(q_1 + roll)*cos(pitch)*cos(q_3))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2) + (-sin(q_3)*sin(q_1 + roll)*cos(pitch)*cos(q_2) + cos(pitch)*cos(q_3)*cos(q_1 + roll))*(sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2))*cos(pitch)*cos(yaw) - sin(q_2)*sin(yaw)*sin(q_1 + roll)*cos(pitch)/sqrt(1 - (sin(pitch)*cos(q_2) + sin(q_2)*cos(pitch)*cos(q_1 + roll))**2)
    jacobian[3,7]=((-sin(pitch)*sin(q_3)*cos(q_2) - sin(q_2)*sin(q_3)*cos(pitch)*cos(q_1 + roll))*(sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2) + (sin(pitch)*cos(q_2)*cos(q_3) + sin(q_2)*cos(pitch)*cos(q_3)*cos(q_1 + roll))*(sin(pitch)*sin(q_2)*sin(q_3) - sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) - sin(q_1 + roll)*cos(pitch)*cos(q_3))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2))*cos(pitch)*cos(yaw) + (-sin(pitch)*sin(q_2) + cos(pitch)*cos(q_2)*cos(q_1 + roll))*sin(yaw)/sqrt(1 - (sin(pitch)*cos(q_2) + sin(q_2)*cos(pitch)*cos(q_1 + roll))**2)
    jacobian[3,8]=((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))*(sin(pitch)*sin(q_2)*sin(q_3) - sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) - sin(q_1 + roll)*cos(pitch)*cos(q_3))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2) + (-sin(pitch)*sin(q_2)*cos(q_3) - sin(q_3)*sin(q_1 + roll)*cos(pitch) + cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))*(sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2))*cos(pitch)*cos(yaw)
    jacobian[4,4]=((sin(pitch)*sin(q_2)*sin(q_3) - sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) - sin(q_1 + roll)*cos(pitch)*cos(q_3))*(-sin(pitch)*sin(q_3)*sin(q_1 + roll) + sin(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll) + sin(q_2)*cos(pitch)*cos(q_3))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2) + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))*(-sin(pitch)*sin(q_3)*cos(q_2)*cos(q_1 + roll) - sin(pitch)*sin(q_1 + roll)*cos(q_3) - sin(q_2)*sin(q_3)*cos(pitch))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2))*sin(yaw)*cos(pitch) - (-sin(pitch)*sin(q_2)*cos(q_1 + roll) + cos(pitch)*cos(q_2))*cos(yaw)/sqrt(1 - (sin(pitch)*cos(q_2) + sin(q_2)*cos(pitch)*cos(q_1 + roll))**2)
    jacobian[4,5]=((sin(q_3)*cos(pitch)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_2)*cos(q_3))*(sin(pitch)*sin(q_2)*sin(q_3) - sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) - sin(q_1 + roll)*cos(pitch)*cos(q_3))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2) + (-sin(q_3)*sin(q_1 + roll)*cos(pitch)*cos(q_2) + cos(pitch)*cos(q_3)*cos(q_1 + roll))*(sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2))*sin(yaw)*cos(pitch) + sin(q_2)*sin(q_1 + roll)*cos(pitch)*cos(yaw)/sqrt(1 - (sin(pitch)*cos(q_2) + sin(q_2)*cos(pitch)*cos(q_1 + roll))**2)
    jacobian[4,6]=((sin(q_3)*cos(pitch)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_2)*cos(q_3))*(sin(pitch)*sin(q_2)*sin(q_3) - sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) - sin(q_1 + roll)*cos(pitch)*cos(q_3))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2) + (-sin(q_3)*sin(q_1 + roll)*cos(pitch)*cos(q_2) + cos(pitch)*cos(q_3)*cos(q_1 + roll))*(sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2))*sin(yaw)*cos(pitch) + sin(q_2)*sin(q_1 + roll)*cos(pitch)*cos(yaw)/sqrt(1 - (sin(pitch)*cos(q_2) + sin(q_2)*cos(pitch)*cos(q_1 + roll))**2)
    jacobian[4,7]=((-sin(pitch)*sin(q_3)*cos(q_2) - sin(q_2)*sin(q_3)*cos(pitch)*cos(q_1 + roll))*(sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2) + (sin(pitch)*cos(q_2)*cos(q_3) + sin(q_2)*cos(pitch)*cos(q_3)*cos(q_1 + roll))*(sin(pitch)*sin(q_2)*sin(q_3) - sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) - sin(q_1 + roll)*cos(pitch)*cos(q_3))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2))*sin(yaw)*cos(pitch) - (-sin(pitch)*sin(q_2) + cos(pitch)*cos(q_2)*cos(q_1 + roll))*cos(yaw)/sqrt(1 - (sin(pitch)*cos(q_2) + sin(q_2)*cos(pitch)*cos(q_1 + roll))**2)
    jacobian[4,8]=((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))*(sin(pitch)*sin(q_2)*sin(q_3) - sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) - sin(q_1 + roll)*cos(pitch)*cos(q_3))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2) + (-sin(pitch)*sin(q_2)*cos(q_3) - sin(q_3)*sin(q_1 + roll)*cos(pitch) + cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))*(sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2))*sin(yaw)*cos(pitch)
    jacobian[5,3]=(-sin(pitch)*sin(q_2)*sin(yaw)*cos(q_1 + roll) + sin(q_2)*sin(q_1 + roll)*cos(yaw) + sin(yaw)*cos(pitch)*cos(q_2))**2/((sin(pitch)*sin(q_2)*sin(yaw)*cos(q_1 + roll) - sin(q_2)*sin(q_1 + roll)*cos(yaw) - sin(yaw)*cos(pitch)*cos(q_2))**2 + (sin(pitch)*sin(q_2)*cos(yaw)*cos(q_1 + roll) + sin(q_2)*sin(yaw)*sin(q_1 + roll) - cos(pitch)*cos(q_2)*cos(yaw))**2) + (sin(pitch)*sin(q_2)*cos(yaw)*cos(q_1 + roll) + sin(q_2)*sin(yaw)*sin(q_1 + roll) - cos(pitch)*cos(q_2)*cos(yaw))**2/((sin(pitch)*sin(q_2)*sin(yaw)*cos(q_1 + roll) - sin(q_2)*sin(q_1 + roll)*cos(yaw) - sin(yaw)*cos(pitch)*cos(q_2))**2 + (sin(pitch)*sin(q_2)*cos(yaw)*cos(q_1 + roll) + sin(q_2)*sin(yaw)*sin(q_1 + roll) - cos(pitch)*cos(q_2)*cos(yaw))**2)
    jacobian[5,4]=-((sin(pitch)*sin(q_2)*sin(q_3) - sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) - sin(q_1 + roll)*cos(pitch)*cos(q_3))*(-sin(pitch)*sin(q_3)*sin(q_1 + roll) + sin(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll) + sin(q_2)*cos(pitch)*cos(q_3))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2) + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))*(-sin(pitch)*sin(q_3)*cos(q_2)*cos(q_1 + roll) - sin(pitch)*sin(q_1 + roll)*cos(q_3) - sin(q_2)*sin(q_3)*cos(pitch))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2))*sin(pitch) + (sin(pitch)*sin(yaw)*cos(q_2) + sin(q_2)*sin(yaw)*cos(pitch)*cos(q_1 + roll))*(sin(pitch)*sin(q_2)*cos(yaw)*cos(q_1 + roll) + sin(q_2)*sin(yaw)*sin(q_1 + roll) - cos(pitch)*cos(q_2)*cos(yaw))/((sin(pitch)*sin(q_2)*sin(yaw)*cos(q_1 + roll) - sin(q_2)*sin(q_1 + roll)*cos(yaw) - sin(yaw)*cos(pitch)*cos(q_2))**2 + (sin(pitch)*sin(q_2)*cos(yaw)*cos(q_1 + roll) + sin(q_2)*sin(yaw)*sin(q_1 + roll) - cos(pitch)*cos(q_2)*cos(yaw))**2) + (sin(pitch)*cos(q_2)*cos(yaw) + sin(q_2)*cos(pitch)*cos(yaw)*cos(q_1 + roll))*(-sin(pitch)*sin(q_2)*sin(yaw)*cos(q_1 + roll) + sin(q_2)*sin(q_1 + roll)*cos(yaw) + sin(yaw)*cos(pitch)*cos(q_2))/((sin(pitch)*sin(q_2)*sin(yaw)*cos(q_1 + roll) - sin(q_2)*sin(q_1 + roll)*cos(yaw) - sin(yaw)*cos(pitch)*cos(q_2))**2 + (sin(pitch)*sin(q_2)*cos(yaw)*cos(q_1 + roll) + sin(q_2)*sin(yaw)*sin(q_1 + roll) - cos(pitch)*cos(q_2)*cos(yaw))**2)
    jacobian[5,5]=-((sin(q_3)*cos(pitch)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_2)*cos(q_3))*(sin(pitch)*sin(q_2)*sin(q_3) - sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) - sin(q_1 + roll)*cos(pitch)*cos(q_3))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2) + (-sin(q_3)*sin(q_1 + roll)*cos(pitch)*cos(q_2) + cos(pitch)*cos(q_3)*cos(q_1 + roll))*(sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2))*sin(pitch) + (-sin(pitch)*sin(q_2)*sin(yaw)*sin(q_1 + roll) - sin(q_2)*cos(yaw)*cos(q_1 + roll))*(sin(pitch)*sin(q_2)*cos(yaw)*cos(q_1 + roll) + sin(q_2)*sin(yaw)*sin(q_1 + roll) - cos(pitch)*cos(q_2)*cos(yaw))/((sin(pitch)*sin(q_2)*sin(yaw)*cos(q_1 + roll) - sin(q_2)*sin(q_1 + roll)*cos(yaw) - sin(yaw)*cos(pitch)*cos(q_2))**2 + (sin(pitch)*sin(q_2)*cos(yaw)*cos(q_1 + roll) + sin(q_2)*sin(yaw)*sin(q_1 + roll) - cos(pitch)*cos(q_2)*cos(yaw))**2) + (-sin(pitch)*sin(q_2)*sin(q_1 + roll)*cos(yaw) + sin(q_2)*sin(yaw)*cos(q_1 + roll))*(-sin(pitch)*sin(q_2)*sin(yaw)*cos(q_1 + roll) + sin(q_2)*sin(q_1 + roll)*cos(yaw) + sin(yaw)*cos(pitch)*cos(q_2))/((sin(pitch)*sin(q_2)*sin(yaw)*cos(q_1 + roll) - sin(q_2)*sin(q_1 + roll)*cos(yaw) - sin(yaw)*cos(pitch)*cos(q_2))**2 + (sin(pitch)*sin(q_2)*cos(yaw)*cos(q_1 + roll) + sin(q_2)*sin(yaw)*sin(q_1 + roll) - cos(pitch)*cos(q_2)*cos(yaw))**2)
    jacobian[5,6]=-((sin(q_3)*cos(pitch)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_2)*cos(q_3))*(sin(pitch)*sin(q_2)*sin(q_3) - sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) - sin(q_1 + roll)*cos(pitch)*cos(q_3))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2) + (-sin(q_3)*sin(q_1 + roll)*cos(pitch)*cos(q_2) + cos(pitch)*cos(q_3)*cos(q_1 + roll))*(sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2))*sin(pitch) + (-sin(pitch)*sin(q_2)*sin(yaw)*sin(q_1 + roll) - sin(q_2)*cos(yaw)*cos(q_1 + roll))*(sin(pitch)*sin(q_2)*cos(yaw)*cos(q_1 + roll) + sin(q_2)*sin(yaw)*sin(q_1 + roll) - cos(pitch)*cos(q_2)*cos(yaw))/((sin(pitch)*sin(q_2)*sin(yaw)*cos(q_1 + roll) - sin(q_2)*sin(q_1 + roll)*cos(yaw) - sin(yaw)*cos(pitch)*cos(q_2))**2 + (sin(pitch)*sin(q_2)*cos(yaw)*cos(q_1 + roll) + sin(q_2)*sin(yaw)*sin(q_1 + roll) - cos(pitch)*cos(q_2)*cos(yaw))**2) + (-sin(pitch)*sin(q_2)*sin(q_1 + roll)*cos(yaw) + sin(q_2)*sin(yaw)*cos(q_1 + roll))*(-sin(pitch)*sin(q_2)*sin(yaw)*cos(q_1 + roll) + sin(q_2)*sin(q_1 + roll)*cos(yaw) + sin(yaw)*cos(pitch)*cos(q_2))/((sin(pitch)*sin(q_2)*sin(yaw)*cos(q_1 + roll) - sin(q_2)*sin(q_1 + roll)*cos(yaw) - sin(yaw)*cos(pitch)*cos(q_2))**2 + (sin(pitch)*sin(q_2)*cos(yaw)*cos(q_1 + roll) + sin(q_2)*sin(yaw)*sin(q_1 + roll) - cos(pitch)*cos(q_2)*cos(yaw))**2)
    jacobian[5,7]=-((-sin(pitch)*sin(q_3)*cos(q_2) - sin(q_2)*sin(q_3)*cos(pitch)*cos(q_1 + roll))*(sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2) + (sin(pitch)*cos(q_2)*cos(q_3) + sin(q_2)*cos(pitch)*cos(q_3)*cos(q_1 + roll))*(sin(pitch)*sin(q_2)*sin(q_3) - sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) - sin(q_1 + roll)*cos(pitch)*cos(q_3))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2))*sin(pitch) + (-sin(pitch)*sin(q_2)*sin(yaw)*cos(q_1 + roll) + sin(q_2)*sin(q_1 + roll)*cos(yaw) + sin(yaw)*cos(pitch)*cos(q_2))*(sin(pitch)*cos(q_2)*cos(yaw)*cos(q_1 + roll) + sin(q_2)*cos(pitch)*cos(yaw) + sin(yaw)*sin(q_1 + roll)*cos(q_2))/((sin(pitch)*sin(q_2)*sin(yaw)*cos(q_1 + roll) - sin(q_2)*sin(q_1 + roll)*cos(yaw) - sin(yaw)*cos(pitch)*cos(q_2))**2 + (sin(pitch)*sin(q_2)*cos(yaw)*cos(q_1 + roll) + sin(q_2)*sin(yaw)*sin(q_1 + roll) - cos(pitch)*cos(q_2)*cos(yaw))**2) + (sin(pitch)*sin(q_2)*cos(yaw)*cos(q_1 + roll) + sin(q_2)*sin(yaw)*sin(q_1 + roll) - cos(pitch)*cos(q_2)*cos(yaw))*(sin(pitch)*sin(yaw)*cos(q_2)*cos(q_1 + roll) + sin(q_2)*sin(yaw)*cos(pitch) - sin(q_1 + roll)*cos(q_2)*cos(yaw))/((sin(pitch)*sin(q_2)*sin(yaw)*cos(q_1 + roll) - sin(q_2)*sin(q_1 + roll)*cos(yaw) - sin(yaw)*cos(pitch)*cos(q_2))**2 + (sin(pitch)*sin(q_2)*cos(yaw)*cos(q_1 + roll) + sin(q_2)*sin(yaw)*sin(q_1 + roll) - cos(pitch)*cos(q_2)*cos(yaw))**2)
    jacobian[5,8]=-((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))*(sin(pitch)*sin(q_2)*sin(q_3) - sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) - sin(q_1 + roll)*cos(pitch)*cos(q_3))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2) + (-sin(pitch)*sin(q_2)*cos(q_3) - sin(q_3)*sin(q_1 + roll)*cos(pitch) + cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))*(sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))/((-sin(pitch)*sin(q_2)*sin(q_3) + sin(q_3)*cos(pitch)*cos(q_2)*cos(q_1 + roll) + sin(q_1 + roll)*cos(pitch)*cos(q_3))**2 + (sin(pitch)*sin(q_2)*cos(q_3) + sin(q_3)*sin(q_1 + roll)*cos(pitch) - cos(pitch)*cos(q_2)*cos(q_3)*cos(q_1 + roll))**2))*sin(pitch)

    # We set XY to be uncontrolled to allow obtaining body rate reference
    J_G_controlled = jacobian[:,2:9]
    J_G_uncontrolled = jacobian[:,0:2]

    # Transformation between euler angle time derivatives and angular velocity
    T_euler = Matrix([[0, -sin(yaw), cos(yaw)*cos(pitch)],
                        [0, cos(yaw), sin(yaw)*cos(pitch)],
                        [1, 0, -sin(pitch)]])

    T_A = Matrix([[eye(3), zeros(3, 3)],
                    [zeros(3, 3), T_euler]])
    
    R_Ie = Matrix([[-(sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*sin(q_1)*sin(q_2) + (sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw))*sin(q_2)*cos(q_1) - cos(pitch)*cos(q_2)*cos(yaw), (sin(q_1)*cos(q_3) + sin(q_3)*cos(q_1)*cos(q_2))*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + (sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*(-sin(q_1)*sin(q_3)*cos(q_2) + cos(q_1)*cos(q_3)) + sin(q_2)*sin(q_3)*cos(pitch)*cos(yaw), (sin(q_1)*sin(q_3) - cos(q_1)*cos(q_2)*cos(q_3))*(sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw)) + (sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll))*(sin(q_1)*cos(q_2)*cos(q_3) + sin(q_3)*cos(q_1)) - sin(q_2)*cos(pitch)*cos(q_3)*cos(yaw)], [-(sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*sin(q_1)*sin(q_2) + (sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw))*sin(q_2)*cos(q_1) - sin(yaw)*cos(pitch)*cos(q_2), (sin(q_1)*cos(q_3) + sin(q_3)*cos(q_1)*cos(q_2))*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + (sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*(-sin(q_1)*sin(q_3)*cos(q_2) + cos(q_1)*cos(q_3)) + sin(q_2)*sin(q_3)*sin(yaw)*cos(pitch), (sin(q_1)*sin(q_3) - cos(q_1)*cos(q_2)*cos(q_3))*(sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw)) + (sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw))*(sin(q_1)*cos(q_2)*cos(q_3) + sin(q_3)*cos(q_1)) - sin(q_2)*sin(yaw)*cos(pitch)*cos(q_3)], [sin(pitch)*cos(q_2) - sin(q_1)*sin(q_2)*sin(roll)*cos(pitch) + sin(q_2)*cos(pitch)*cos(q_1)*cos(roll), (sin(q_1)*cos(q_3) + sin(q_3)*cos(q_1)*cos(q_2))*cos(pitch)*cos(roll) + (-sin(q_1)*sin(q_3)*cos(q_2) + cos(q_1)*cos(q_3))*sin(roll)*cos(pitch) - sin(pitch)*sin(q_2)*sin(q_3), (sin(q_1)*sin(q_3) - cos(q_1)*cos(q_2)*cos(q_3))*cos(pitch)*cos(roll) + (sin(q_1)*cos(q_2)*cos(q_3) + sin(q_3)*cos(q_1))*sin(roll)*cos(pitch) + sin(pitch)*sin(q_2)*cos(q_3)]])

    def __init__(self):
        self.jfunc = lambdify((self.yaw, self.pitch, self.roll, self.q_1, self.q_2, self.q_3), self.jacobian, modules='numpy')

        self.J_G_controlled_func = lambdify((self.yaw, self.pitch, self.roll, self.q_1, self.q_2, self.q_3), self.J_G_controlled, modules='numpy')
        self.J_G_uncontrolled_func = lambdify((self.yaw, self.pitch, self.roll, self.q_1, self.q_2, self.q_3), self.J_G_uncontrolled, modules='numpy')

        self.T_A_func = lambdify((self.yaw, self.pitch), self.T_A, modules='numpy')

        self.R_Ie_func = lambdify((self.yaw, self.pitch, self.roll, self.q_1, self.q_2, self.q_3), self.R_Ie, modules='numpy')

        self.state = [0., 0., 0., 0., 0., 0.]
    
    def set_state(self, new_state : dict):
        """
        Set the state variables with a list or NumPy array of states.

        Parameters:
        - new_state: Dict of states.

        """
        self.state[0] = new_state['yaw']
        self.state[1] = new_state['pitch']
        self.state[2] = new_state['roll']
        self.state[3] = new_state['q1']
        self.state[4] = new_state['q2']
        self.state[5] = new_state['q3']

    def evaluate_full_jacobian(self):
        """
        Evaluate the expression with a list or NumPy array of states.


        Returns:
        - The evaluated result as a NumPy array.
        """
        return self.jfunc(*self.state)

    def evaluate_controlled_jacobian(self):
        """
        Evaluate the controlled part of the jacobian with a list or NumPy array of states.

        Returns:
        - The evaluated result as a NumPy array.
        """
        return self.J_G_controlled_func(*self.state)
    
    def evaluate_uncontrolled_jacobian(self):
        """
        Evaluate the uncontrolled part of the jacobian with a list or NumPy array of states.

        Returns:
        - The evaluated result as a NumPy array.
        """
        return self.J_G_uncontrolled_func(*self.state)
    
    def evaluate_pseudoinverse_controlled_jacobian(self):
        """
        Evaluate the pseudoinverse of the jacobian with a list or NumPy array of states.

        Returns:
        - The evaluated result as a NumPy array.
        """
        J = self.evaluate_controlled_jacobian(self.state)
        J_pinv = np.matmul(J.transpose(),np.linalg.inv(np.matmul(J,J.transpose())))
        return J_pinv
    
    def evaluate_angular_velocity_transformation(self):
        """
        Evaluate the transformation matrix between euler angle time derivatives and angular velocity with a list or NumPy array of states.

        Parameters:
        - values: List or NumPy array of states (length 2).

        Returns:
        - The evaluated result as a NumPy array.
        """
        return self.T_A_func(self.state[0], self.state[1])

    def evaluate_rotation_matrix(self):
        """
        Evaluate the rotation matrix between the inertial and end-effector frame with a list or NumPy array of states.

        Parameters:
        - values: List or NumPy array of states (length 6).

        Returns:
        - The evaluated result as a NumPy array.
        """
        return self.R_Ie_func(*self.state)
    


class ManipulatorJacobian:
    q_1, q_2, q_3 = symbols('q_1 q_2 q_3')
    # This geometric jacobian was derived in the am_kinematics repository with sympy.
    jacobian = zeros(6, 3)

    jacobian[0,1]=-(0.273*cos(q_3) + 0.311)*cos(q_2)
    jacobian[0,2]=0.273*sin(q_2)*sin(q_3)
    jacobian[1,0]=-0.273*sin(q_1)*sin(q_3) + 0.273*cos(q_1)*cos(q_2)*cos(q_3) + 0.311*cos(q_1)*cos(q_2) + 0.11*cos(q_1)
    jacobian[1,1]=-0.273*sin(q_1)*sin(q_2)*cos(q_3) - 0.311*sin(q_1)*sin(q_2)
    jacobian[1,2]=-0.273*sin(q_1)*sin(q_3)*cos(q_2) + 0.273*cos(q_1)*cos(q_3)
    jacobian[2,0]=0.273*sin(q_1)*cos(q_2)*cos(q_3) + 0.311*sin(q_1)*cos(q_2) + 0.11*sin(q_1) + 0.273*sin(q_3)*cos(q_1)
    jacobian[2,1]=0.273*sin(q_2)*cos(q_1)*cos(q_3) + 0.311*sin(q_2)*cos(q_1)
    jacobian[2,2]=0.273*sin(q_1)*cos(q_3) + 0.273*sin(q_3)*cos(q_1)*cos(q_2)
    jacobian[3,0]=sin(q_2)*cos(q_1)*cos(q_2)/(sin(q_1)**2*sin(q_2)**2 + cos(q_2)**2)
    jacobian[3,1]=sin(q_1)*sin(q_2)**2/(sin(q_1)**2*sin(q_2)**2 + cos(q_2)**2) + sin(q_1)*cos(q_2)**2/(sin(q_1)**2*sin(q_2)**2 + cos(q_2)**2)
    jacobian[4,0]=sin(q_1)*sin(q_2)/sqrt(-sin(q_2)**2*cos(q_1)**2 + 1)
    jacobian[4,1]=-cos(q_1)*cos(q_2)/sqrt(-sin(q_2)**2*cos(q_1)**2 + 1)
    jacobian[5,0]=(sin(q_1)*sin(q_3) - cos(q_1)*cos(q_2)*cos(q_3))*(-sin(q_1)*sin(q_3)*cos(q_2) + cos(q_1)*cos(q_3))/((sin(q_1)*sin(q_3) - cos(q_1)*cos(q_2)*cos(q_3))**2 + (sin(q_1)*cos(q_3) + sin(q_3)*cos(q_1)*cos(q_2))**2) + (-sin(q_1)*cos(q_3) - sin(q_3)*cos(q_1)*cos(q_2))*(sin(q_1)*cos(q_2)*cos(q_3) + sin(q_3)*cos(q_1))/((sin(q_1)*sin(q_3) - cos(q_1)*cos(q_2)*cos(q_3))**2 + (sin(q_1)*cos(q_3) + sin(q_3)*cos(q_1)*cos(q_2))**2)
    jacobian[5,1]=-(sin(q_1)*sin(q_3) - cos(q_1)*cos(q_2)*cos(q_3))*sin(q_2)*sin(q_3)*cos(q_1)/((sin(q_1)*sin(q_3) - cos(q_1)*cos(q_2)*cos(q_3))**2 + (sin(q_1)*cos(q_3) + sin(q_3)*cos(q_1)*cos(q_2))**2) + (-sin(q_1)*cos(q_3) - sin(q_3)*cos(q_1)*cos(q_2))*sin(q_2)*cos(q_1)*cos(q_3)/((sin(q_1)*sin(q_3) - cos(q_1)*cos(q_2)*cos(q_3))**2 + (sin(q_1)*cos(q_3) + sin(q_3)*cos(q_1)*cos(q_2))**2)
    jacobian[5,2]=(-sin(q_1)*sin(q_3) + cos(q_1)*cos(q_2)*cos(q_3))*(sin(q_1)*sin(q_3) - cos(q_1)*cos(q_2)*cos(q_3))/((sin(q_1)*sin(q_3) - cos(q_1)*cos(q_2)*cos(q_3))**2 + (sin(q_1)*cos(q_3) + sin(q_3)*cos(q_1)*cos(q_2))**2) + (-sin(q_1)*cos(q_3) - sin(q_3)*cos(q_1)*cos(q_2))*(sin(q_1)*cos(q_3) + sin(q_3)*cos(q_1)*cos(q_2))/((sin(q_1)*sin(q_3) - cos(q_1)*cos(q_2)*cos(q_3))**2 + (sin(q_1)*cos(q_3) + sin(q_3)*cos(q_1)*cos(q_2))**2)
    
    R_be = Matrix([[-cos(q_2), sin(q_2)*sin(q_3), -sin(q_2)*cos(q_3)], [-sin(q_1)*sin(q_2), -sin(q_1)*sin(q_3)*cos(q_2) + cos(q_1)*cos(q_3), sin(q_1)*cos(q_2)*cos(q_3) + sin(q_3)*cos(q_1)], [sin(q_2)*cos(q_1), sin(q_1)*cos(q_3) + sin(q_3)*cos(q_1)*cos(q_2), sin(q_1)*sin(q_3) - cos(q_1)*cos(q_2)*cos(q_3)]])
    
    def __init__(self):
        self.jfunc = lambdify((self.q_1, self.q_2, self.q_3), self.jacobian, modules='numpy')

        self.R_be_func = lambdify((self.q_1, self.q_2, self.q_3), self.R_be, modules='numpy')

        self.state = [0., 0., 0.]

    def evaluate_jacobian(self):
        """
        Evaluate the expression with a list or NumPy array of states.


        Returns:
        - The evaluated result as a NumPy array.
        """
        return self.jfunc(*self.state)

    def evaluate_pseudoinverse_jacobian(self):
        """
        Evaluate the pseudoinverse of the jacobian with a list or NumPy array of states.

        Returns:
        - The evaluated result as a NumPy array.
        """
        J = self.evaluate_jacobian(self.state)
        J_pinv = np.linalg.pinv(J)
        return J_pinv
    
    def evaluate_rotation_matrix(self):
        """
        Evaluate the rotation matrix between the base and end-effector frame with a list or NumPy array of states.

        Parameters:
        - values: List or NumPy array of states (length 3).

        Returns:
        - The evaluated result as a NumPy array.
        """
        return self.R_be_func(*self.state)

    def set_state(self, new_state : dict):
        """
        Set the state variables with a dictof states.

        Parameters:
        - new_state: Dict of states.

        """
        self.state[0] = new_state['q1']
        self.state[1] = new_state['q2']
        self.state[2] = new_state['q3']