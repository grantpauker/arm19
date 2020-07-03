#!/usr/bin/env python
import control
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation, patches
from scipy import integrate

np.set_printoptions(suppress=True)

def cartpend(x, m, M, L, g, d, u):
    cy = np.cos(x[2])
    sy = np.sin(x[2])

    D = m * (L**2) * (M+m*(1-cy**2))

    x_dot = np.array([0.0, 0.0, 0.0, 0.0], dtype='f')

    x_dot[0] = x[1]
    x_dot[1] = (1/D)*(-m*m*L*L*g*cy*sy + m*L*L *
                      (m*L*x[3]*x[3]*sy - d*x[1])) + m*L*L*(1/D)*u
    x_dot[2] = x[3]
    x_dot[3] = (1/D)*((m+M)*m*g*L*sy - m*L*cy*(m*L*x[3]*x[3]*sy -
                                               d*x[1])) - m*L*cy*(1/D)*u + 2*np.random.normal()

    return x_dot


CART_W = 0.2  # width of the cart to draw
CART_H = 0.2  # height of the cart to draw

# m = 0.2  # mass of bob
# M = 1  # mass of cart
# L = 0.6  # length of pendulum
# g = -9.81  # gravity
# d = 1  # damping

m = 1  # mass of bob
M = 5  # mass of cart
L = 2  # length of pendulum
g = -10  # gravity
d = 1  # damping

s = 1  # pendulum up (s=1)


# state space matricies
A = [[0, 1, 0, 0],
     [0, - d/M, - m*g/M, 0],
     [0, 0, 0, 1],
     [0, - s*d/(M*L), - s*(m+M)*g/(M*L), 0]
     ]

B = [[0],
     [1/M],
     [0],
     [s*1/(M*L)]]

C = np.array([[1, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0]])


D = np.zeros([4, 1])

# example poles
# # # # _p = [-.01, -.02, -.03, -.04]  # not enough
# _p = [-.3, -.4, -.5, -.6]  # just barely
_p = [-1, -1.1, -1.2, -1.3]  # good
# _p = [-2, -2.1, -2.2, -2.3]  # aggressive
# _p = [-3.0, -3.1, -3.2, -3.3]  # aggressive
# _p = [-3.5, -3.6, -3.7, -3.8]  # breaks
# _p = [-10,-10.1,-10.2,-10.3]  # breaks
# _p = [-10,-10.1,-10.2,-10.3]  # breaks

p = np.array(_p)

x0 = np.array([0, 0, -np.pi+0.1, 1])  # intial x

Q = np.array([[10, 0, 0, 0], [0, 1, 0, 0], [0, 0, 2, 0], [0, 0, 0, 2]])
R = 0.1

# K = control.place(A, B, p)
K = control.lqr(A, B, Q, R)[0]
# controllability = control.ctrb(A, B)
observability = control.obsv(A, C)
print(observability)
sys = control.ss(A, B, C, D)


def xDot(t, y):
    # print(t)
    error = y-[0, 0, -np.pi, 0]
    u = -K * ((error)[np.newaxis].T)
    return cartpend(y, m, M, L, g, d, u[0][0])


t0 = 0
t1 = 2
dt = 0.02

integration = integrate.solve_ivp(
    xDot, [t0, t1], x0, t_eval=np.arange(t0, t1, dt))
states = integration.y.T

fig = plt.figure()
ax = plt.axes(xlim=(-2, 2), ylim=(0, 2), aspect='equal')
# plt.gca().set_aspect('equal', adjustable='box')
line, = ax.plot([], [], color='k', linewidth=3)


def init():
    line.set_data([], [])
    return line,


def animate(i):
    state = states[i]
    angle = state[2] - np.pi/2
    pos = np.array([state[0], 0.1]) - [CART_W/2, CART_H/2]
    bob = pos + np.array([L*np.cos(angle), L*np.sin(angle)])
    points = np.array([pos, bob]) + [CART_W/2, CART_H/2]
    x, y = points.T
    ptch = []
    line.set_data(x, y)
    ptch.append(ax.add_patch(patches.Circle(points[1], 0.05)))
    ptch.append(ax.add_patch(patches.Rectangle(
        pos, CART_W, CART_H, angle=0.0)))
    ptch.append(line)
    return ptch


anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=int(t1/dt), interval=dt * 1000, blit=True)

plt.show()
