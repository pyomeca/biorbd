import biorbd
import math

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# functions for cylinder rotation
def cyl2cart(r, theta, z):
    return (r * np.cos(theta), r * np.sin(theta), z)


def roll(R, zi, zf, RT):
    t = np.arange(0, 2 * np.pi + 0.1, 0.1)
    z = np.array([zi, zf])
    t, z = np.meshgrid(t, z)
    p, q = t.shape
    r = R * np.ones([p, q], float)
    # cylindrical coordinates to Cartesian coordinate
    x, y, z = cyl2cart(r, t, z)

    # Euler rotation
    rot = np.dot(RT.rot().to_array(), np.array([x.ravel(), y.ravel(), z.ravel()]))

    x_rt = rot[0, :].reshape(x.shape) + RTw.trans().to_array()[0]
    y_rt = rot[1, :].reshape(y.shape) + RTw.trans().to_array()[1]
    z_rt = rot[2, :].reshape(z.shape) + RTw.trans().to_array()[2]

    return (
        x_rt,
        y_rt,
        z_rt,
    )


# model
model = biorbd.Model("WrappingObjectExample.bioMod")
# wrapping RT matrix
RTw = biorbd.WrappingHalfCylinder(model.muscle(0).pathModifier().object(0)).RT(
    model, np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
)
RTw_trans = RTw.transpose()

# Point in global base
Po = [0.264534159209888, 0.08668973339838759, 0.4024172540921048]
Pi = [0.24035592212771023, 0.36899416468322876, 0.8650273094461659]
Pi_wrap = [0.28997869688863276, 0.34739632020407846, 0.6636920365713757]
Po_wrap = [0.29330355695193333, 0.3289556960274852, 0.627410425452938]

# Plot the surface
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
x, y, z = roll(0.05, -0.2, 0.2, RTw)
ax.plot_surface(x, y, z)
plt.plot((Po[0], Po_wrap[0]), (Po[1], Po_wrap[1]), (Po[2], Po_wrap[2]), marker="o")
plt.plot((Pi[0], Pi_wrap[0]), (Pi[1], Pi_wrap[1]), (Pi[2], Pi_wrap[2]), marker="o")

# To set Axe3D to (almost) equal
max_range = np.array([x.max() - x.min(), y.max() - y.min(), z.max() - z.min()]).max()
Xb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][0].flatten() + 0.5 * (x.max() + x.min())
Yb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][1].flatten() + 0.5 * (y.max() + y.min())
Zb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][2].flatten() + 0.5 * (z.max() + z.min())
for xb, yb, zb in zip(Xb, Yb, Zb):
    ax.plot([xb], [yb], [zb], "w")
plt.show()

# Compute muscle length
r = biorbd.WrappingHalfCylinder(model.muscle(0).pathModifier().object(0)).radius()
l_muscle_bd = model.muscle(0).musculoTendonLength(model, np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]))


l_wt_arc = np.sqrt((Po_wrap[0] - Po[0]) ** 2 + (Po_wrap[1] - Po[1]) ** 2 + (Po_wrap[2] - Po[2]) ** 2) + np.sqrt(
    (Pi_wrap[0] - Pi[0]) ** 2 + (Pi_wrap[1] - Pi[1]) ** 2 + (Pi_wrap[2] - Pi[2]) ** 2
)

# Pi_wrap and Po_wrap provide in wrapping base to compute arc length
Pi_wrap = biorbd.Vector3d(0.28997869688863276, 0.34739632020407846, 0.6636920365713757)
Pi_wrap.applyRT(RTw_trans)
Pi_wrap = Pi_wrap.to_array()

Po_wrap = biorbd.Vector3d(0.29330355695193333, 0.3289556960274852, 0.627410425452938)
Po_wrap.applyRT(RTw_trans)
Po_wrap = Po_wrap.to_array()

arc = (
    math.acos(
        ((Po_wrap[0] * Pi_wrap[0]) + (Pi_wrap[1] * Po_wrap[1]))
        / (math.sqrt(((Pi_wrap[1] ** 2) + (Pi_wrap[0] ** 2)) * ((Po_wrap[1] ** 2) + (Po_wrap[0] ** 2))))
    )
    * r
)

l_arc = math.sqrt(arc**2 + (Po_wrap[2] - Pi_wrap[2]) ** 2)

l_m = l_wt_arc + l_arc  # l_m = 0.582246096069153
