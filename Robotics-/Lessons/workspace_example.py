import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
import matplotlib.pyplot as plt

### two link scara mechanism workspace representation 
# link lengths 
l1 = 1
l2 = 1.5
# number of samples
samp = 500
# all possible theta values
theta1 = np.linspace(0,210,samp)*np.pi/180
theta2 = np.linspace(0,180,samp)*np.pi/180
# generate a grid of theta values 
th1,th2 = np.meshgrid(theta1,theta2, indexing='ij')
# functions to calculate the x and y coordinates for four links with variable angles 
X= l1*np.cos(th1)+l2*np.cos(th1+th2)
Y= l1*np.sin(th1)+l2*np.sin(th1+th2)
#plot results
# plt.figure(1)
# plt.title('2 link SCARA mechanism workspace')
# plt.plot(X,Y,'b.')
# plt.ylabel('y')
# plt.xlabel('x')


# 3 link mechanism workspace representation 
l11 = 1
l22 = 1.5
l33 = 1.25
# number of samples
samp = 10
# all possible theta values
theta11 = np.linspace(-45,130,samp)*np.pi/180
theta22 = np.linspace(0,90,samp)*np.pi/180
theta33 = np.linspace(0,80,samp)*np.pi/180
# generate a grid of theta values 
th11,th22,th33 = np.meshgrid(theta11,theta22,theta33, indexing='ij')
# functions to calculate the x and y coordinates for four links with variable angles 
X1= l11*np.cos(th11) + l22*np.cos(th11+th22) + l33*np.cos(th11+th22+th33)
Y1= l11*np.sin(th11) + l22*np.sin(th11+th22) + l33*np.sin(th11+th22+th33)

#plot results
# plt.figure(2)
# plt.title('3 link mechanism workspace') 
# plt.plot(X1.flatten(),Y1.flatten(),'b.')
# plt.ylabel('y')
# plt.xlabel('x')
# plt.show()

# understanding trapzeodial function 
f = rtb.trapezoidal_func(1, 2, 5, 0.2)
f1 = rtb.trapezoidal_func(1, 2, 5)
print("trapezoidal function",f )
t = np.linspace(0,10,40)
X = np.array(f(t))
X1 = np.array(f1(t))
plt.figure()
plt.title("trapezoidal function position")
plt.plot(t,X[0,:],  label="with max_speed")
plt.plot(t,X1[0,:], label="without max speed")
plt.legend()

plt.figure()
plt.title("trapezoidal function velocity")
plt.plot(t,X[1,:] ,label="with max_speed" )
plt.plot(t,X1[1,:], label="without max speed")
plt.legend()

plt.figure()
plt.title("trapezoidal function acceleration")
plt.plot(t,X[2,:],  label="with max_speed")
plt.plot(t,X1[2,:], label="without max speed")
plt.legend()
plt.show()
