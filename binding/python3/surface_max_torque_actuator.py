from .biorbd import currentLinearAlgebraBackend

import numpy as np

try:
    from matplotlib import pyplot as plt

    matplotlib_found = True
except ModuleNotFoundError:
    matplotlib_found = False
if currentLinearAlgebraBackend() == 1:
    from casadi import Function, MX


def surface_max_torque_actuator(model, dof, resolution=40, convert_to_degree=False):
    if not matplotlib_found:
        raise ModuleNotFoundError("matplotlib must be installed to use biorbd.surface_max_torque_actuator")

    if convert_to_degree:
        d2r = np.pi / 180
    else:
        d2r = 1
    min_bound_q = -200 * d2r
    max_bound_q = 200 * d2r
    min_bound_qdot = -500 * d2r
    max_bound_qdot = 500 * d2r
    nbq = model.nbQ()

    if currentLinearAlgebraBackend() == 1:
        torque_act = MX.sym("act", nbq, 1)
        q_sym = MX.sym("q", nbq, 1)
        qdot_sym = MX.sym("q_dot", nbq, 1)
        torque_func = Function(
            "torque_func",
            [torque_act, q_sym, qdot_sym],
            [model.torque(torque_act, q_sym, qdot_sym).to_mx()],
            ["activation", "Q", "Qdot"],
            ["Tau"],
        )
    else:
        torque_func = model.torque

    max_act = np.ones(nbq)
    q = np.arange(min_bound_q, max_bound_q, (max_bound_q - min_bound_q) / resolution)
    qdot = np.arange(min_bound_qdot, max_bound_qdot, (max_bound_qdot - min_bound_qdot) / resolution)

    tau_pos = np.zeros((resolution, resolution))
    tau_neg = np.zeros((resolution, resolution))
    for i in range(resolution):
        for j in range(resolution):
            pos = torque_func(max_act, np.ones(nbq) * q[i], np.ones(nbq) * qdot[j])
            neg = torque_func(-max_act, np.ones(nbq) * q[i], np.ones(nbq) * qdot[j])
            if currentLinearAlgebraBackend() == 0:
                pos = pos.to_array()
                neg = neg.to_array()
            tau_pos[i, j] = pos[dof]
            tau_neg[i, j] = neg[dof]

    q = q / d2r
    qdot = qdot / d2r
    q, qdot = np.meshgrid(qdot, q)

    def plot_surface(tau):
        fig = plt.figure()
        ax = fig.gca(projection="3d")
        ax.plot_surface(q, qdot, tau)
        ax.set_xlabel("Qdot", fontsize=15)
        ax.set_ylabel("Q", fontsize=15)
        ax.set_zlabel("Tau", fontsize=15)

    plot_surface(tau_pos)
    plot_surface(tau_neg)

    plt.show()
