#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc, animation, rcParams
import sys
font = {'weight': 'bold',
        'size': 22}
rc('font', **font)
rcParams['animation.convert_path'] = '/usr/bin/convert'
rcParams['animation.ffmpeg_path'] = '/usr/bin/ffmpeg'
rpm_per_rps = 60/(2*np.pi)



class ArmSegment:
    def __init__(self, length, constraints):
        self.length = length
        self.constraints = constraints
        self._angle = 0
        self._last_angle = 0
        self.angle = 0

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, angle):
        self._last_angle = self._angle
        self._angle = np.clip(angle, self.constraints[0], self.constraints[1])

    def forwardKinematics(self):
        return self.length * np.array([np.cos(self.angle), np.sin(self.angle)])

    def getDelta(self):
        return self._angle - self._last_angle

    def __str__(self):
        return f"({self.length}, {self.angle})"


class Arm:
    def __init__(self, length, num_segments):
        num_segments = 2
        self.segment_0 = ArmSegment(length, [-np.pi*10, np.pi*10])
        self.segment_1 = ArmSegment(length, [-np.pi*10, np.pi*10])
        self.max_radius = length * num_segments

    def inverseKinematics(self, point):
        point = np.array(point)
        radius = np.linalg.norm(point)

        if radius >= self.max_radius:
            point = (point / radius) * self.max_radius
            # raise ValueError(
            #     f"{radius} is further than arm can reach ({self.max_radius})")

        cos_alpha = (point[0] ** 2 + point[1] ** 2 - self.segment_0.length**2 -
                     self.segment_1.length**2)/(2 * self.segment_0.length * self.segment_1.length)
        alpha = -np.arccos(cos_alpha)

        beta = np.arctan2(point[1], point[0]) - np.arctan2(self.segment_1.length * np.sin(
            alpha), self.segment_0.length + self.segment_1.length*np.cos(alpha))

        self.segment_1.angle = beta + alpha
        self.segment_0.angle = beta
        print(f'{rpm_per_rps*self.segment_0.getDelta()/dt} {rpm_per_rps*self.segment_1.getDelta()/dt}')

    def forwardKinematics(self):
        p0 = self.segment_0.forwardKinematics()
        p1 = p0 + self.segment_1.forwardKinematics()
        return [p0, p1]

    def __str__(self):
        return "(" + ", ".join([str(s) for s in (self.segment_0, self.segment_1)]) + ")"


arm = Arm(5, 2)
2
fig = plt.figure()
ax = plt.axes(xlim=(-12, 12), ylim=(-12, 12), aspect='equal')
# plt.gca().set_aspect('equal', adjustable='box')
line, = ax.plot([], [], color='k', linewidth=3)
t0 = 0
t1 = 1
dt = 0.02


def init():
    line.set_data([], [])
    return line,


states = np.zeros((int((t1-t0)/dt), 2, 2))

for i in range(0, int((t1-t0)/dt)):
    angle = np.deg2rad(i * 360/((t1-t0)/dt))
    end_point = np.array([np.cos(angle), np.sin(angle)]
                         ) * (i/int((t1-t0)/dt)) * 10
    arm.inverseKinematics(end_point)
    forward = arm.forwardKinematics()
    states[i][0] = forward[0]
    states[i][1] = forward[1]
states = np.append(states, np.flip(states, 0), axis=0)


def animate(i):
    joints = np.array([
        [0, 0],
        states[i][0],
        states[i][1],
    ])
    # print(joints)
    x, y = joints.T
    drawn = []
    line.set_data(x, y)
    drawn.append(line)
    # print(i)
    return drawn


anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=2*int(t1/dt), interval=dt * 1000, blit=True)
_writer = animation.writers['ffmpeg']
writer = _writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)
plt.show()
