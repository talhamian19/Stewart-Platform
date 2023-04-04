import numpy as np
import math

Rt = np.array([0, 0, 5]).reshape(3,1)
a = 0*np.pi/180
b = 0*np.pi/180
c = 0*np.pi/180
R = 4.9
r = 5
d = 3
s = 6
Be = np.array([118, 2, -122, 122, -2, -118]).reshape(6,1)*np.pi/180
mu = (40/2)*(np.pi/180)
R1 = np.array([R*np.cos(mu), R*np.sin(mu), 0]).reshape(3,1)
R6 = np.array([R*np.cos(-mu), R*np.sin(-mu), 0]).reshape(3,1)
R2 = np.array([R*np.cos((2*np.pi/3)-mu), R*np.sin((2*np.pi/3)-mu), 0]).reshape(3,1)
R3 = np.array([R*np.cos((2*np.pi/3)+mu), R*np.sin((2*np.pi/3)+mu), 0]).reshape(3,1)
R4 = np.array([R*np.cos((4*np.pi/3)-mu), R*np.sin((4*np.pi/3)-mu), 0]).reshape(3,1)
R5 = np.array([R*np.cos((4*np.pi/3)+mu), R*np.sin((4*np.pi/3)+mu), 0]).reshape(3,1)
Rb = np.hstack((R1, R2, R3, R4, R5, R6))
mu = 25*np.pi/180
rt1 = np.array([r*np.cos((np.pi/3)-mu), r*np.sin((np.pi/3)-mu), 0]).reshape(3,1)
rt2 = np.array([r*np.cos((np.pi/3)+mu), r*np.sin((np.pi/3)+mu), 0]).reshape(3,1)
rt3 = np.array([r*np.cos(np.pi-mu), r*np.sin(np.pi-mu), 0]).reshape(3,1)
rt4 = np.array([r*np.cos(np.pi+mu), r*np.sin(np.pi+mu), 0]).reshape(3,1)
rt5 = np.array([r*np.cos((5*np.pi/3)-mu), r*np.sin((5*np.pi/3)-mu), 0]).reshape(3,1)
rt6 = np.array([r*np.cos((5*np.pi/3)+mu), r*np.sin((5*np.pi/3)+mu), 0]).reshape(3,1)
q = np.hstack((rt1, rt2, rt3, rt4, rt5, rt6))
rotx = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
roty = np.array([[np.cos(b), 0, np.sin(b)], [0, 1, 0], [-np.sin(b), 0, np.cos(b)]])
rotz = np.array([[np.cos(c), np.sin(c), 0], [-np.sin(c), np.cos(c), 0], [0, 0, 1]])
BTR = rotx @ roty @ rotz

r = BTR @ q
Rtp = np.hstack((Rt, Rt, Rt, Rt, Rt, Rt))

L=Rtp+r-Rb
#print(L)
print('\n')
# L = np.zeros((3,6))
# for i in range(3):
#     for j in range(6):
#         q[i, j] = Rt + r[i, j]
#         L[i, j] = q[i, j] - Rb[i, j]
L1, L2, L3, L4, L5, L6 = np.linalg.norm(L, axis=0)
LL = np.array([L1, L2, L3, L4, L5, L6])

# Calculate leg positions
leg1x = [Rb[0, 0], Rt[0] + r[0, 0]]
leg1y = [Rb[1, 0], Rt[1] + r[1, 0]]
leg1z = [Rb[2, 0], Rt[2] + r[2, 0]]
leg2x = [Rb[0, 1], Rt[0] + r[0, 1]]
leg2y = [Rb[1, 1], Rt[1] + r[1, 1]]
leg2z = [Rb[2, 1], Rt[2] + r[2, 1]]
leg3x = [Rb[0, 2], Rt[0] + r[0, 2]]
leg3y = [Rb[1, 2], Rt[1] + r[1, 2]]
leg3z = [Rb[2, 2], Rt[2] + r[2, 2]]
leg4x = [Rb[0, 3], Rt[0] + r[0, 3]]
leg4y = [Rb[1, 3], Rt[1] + r[1, 3]]
leg4z = [Rb[2, 3], Rt[2] + r[2, 3]]
leg5x = [Rb[0, 4], Rt[0] + r[0, 4]]
leg5y = [Rb[1, 4], Rt[1] + r[1, 4]]
leg5z = [Rb[2, 4], Rt[2] + r[2, 4]]
leg6x = [Rb[0, 5], Rt[0] + r[0, 5]]
leg6y = [Rb[1, 5], Rt[1] + r[1, 5]]
leg6z = [Rb[2, 5], Rt[2] + r[2, 5]]

q = Rtp+r
# Calculate servo angles
Lc = LL**2 - s**2 + d**2

M = 2 * d * (np.transpose(q[2, :]) - np.transpose(Rb[2, :]))
#N = 2 * d * ((q[0, :] - Rb[0, :]) * np.cos(Be) + (q[1, :] - Rb[1, :]) * np.sin(Be))

N = np.multiply(2*d,np.subtract((np.multiply((np.subtract(q[0, :].reshape(6,1) , Rb[0, :].reshape(6,1))) , np.cos(Be)) ) , np.multiply((np.subtract(q[1, :].reshape(6,1) , Rb[1, :].reshape(6,1))) , np.sin(Be))))
print("A1",N,'halo \n' )
print("A2",M,'halo \n' )
print("A3",Lc,'halo \n' )

N = N.reshape(1,6)





al = np.arcsin(np.divide(Lc , np.sqrt(np.add(np.square(M) , np.square(N))))) - np.arctan2(N, M)
angles = al[0]
angle1 = math.degrees(angles[0])
angle2 = math.degrees(angles[1])
angle3 = math.degrees(angles[2])
angle4 = math.degrees(angles[3])
angle5 = math.degrees(angles[4])
angle6 = math.degrees(angles[5])
print("Angles: ",angle4)
