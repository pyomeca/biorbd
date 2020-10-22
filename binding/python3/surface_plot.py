def surface_plot(model, dof):
    from casadi import Function, MX
    import numpy as np
    from math import pi

    min_bound_q = -200 / 180 * pi
    max_bound_q = 200 / 180 * pi
    min_bound_qdot = -500 / 180 * pi
    max_bound_qdot = 500 / 180 * pi
    nbq = model.nbQ()

    q_sym = MX.sym("q", nbq, 1)
    qdot_sym = MX.sym("q_dot", nbq, 1)
    torque_act = np.ones(nbq)
    torque_func = Function(
            "torque_func",
            [q_sym, qdot_sym],
            [model.torque(torque_act, q_sym, qdot_sym).to_mx(), model.torque(torque_act * -1, q_sym, qdot_sym).to_mx()],
            ["Q", "Qdot"],
            ["TauMax", "TauMin"])

    q = np.arange(min_bound_q, max_bound_q, (max_bound_q - min_bound_q) / 100)
    qdot = np.arange(min_bound_qdot, max_bound_qdot, (max_bound_qdot - min_bound_qdot) / 100)

    # x = -50
    # y = 150
    # print('q: -50, qdot: 150, tau_min: ', torque_func(np.ones(nbq) * x / 180 * pi, np.ones(nbq) * y / 180 * pi)[1].__array__()[dof])
    # print('q: -50, qdot: 150, tau_max: ', torque_func(np.ones(nbq) * x / 180 * pi, np.ones(nbq) * y / 180 * pi)[0].__array__()[dof])
    # x = -50
    # y = -150
    # print('q: -50, qdot: -150, tau_min: ', torque_func(np.ones(nbq) * x / 180 * pi, np.ones(nbq) * y / 180 * pi)[1].__array__()[dof])
    # print('q: -50, qdot: -150, tau_max: ', torque_func(np.ones(nbq) * x / 180 * pi, np.ones(nbq) * y / 180 * pi)[0].__array__()[dof])
    # x = -50
    # y = 0
    # print('q: -50, qdot: 0, tau_min: ', torque_func(np.ones(nbq) * x / 180 * pi, np.ones(nbq) * y / 180 * pi)[1].__array__()[dof])
    # print('q: -50, qdot: 0, tau_max: ', torque_func(np.ones(nbq) * x / 180 * pi, np.ones(nbq) * y / 180 * pi)[0].__array__()[dof])
    # x = 0
    # y = 0
    # print('q: 0, qdot: 0, tau_min: ', torque_func(np.ones(nbq) * x / 180 * pi, np.ones(nbq) * y / 180 * pi)[1].__array__()[dof])
    # print('q: 0, qdot: 0, tau_max: ', torque_func(np.ones(nbq) * x / 180 * pi, np.ones(nbq) * y / 180 * pi)[0].__array__()[dof])
    # x = 10
    # y = -150
    # print('q: 10, qdot: -150, tau_min: ', torque_func(np.ones(nbq) * x / 180 * pi, np.ones(nbq) * y / 180 * pi)[1].__array__()[dof])
    # print('q: 10, qdot: -150, tau_max: ', torque_func(np.ones(nbq) * x / 180 * pi, np.ones(nbq) * y / 180 * pi)[0].__array__()[dof])

    tau_pos = np.zeros((100, 100))
    tau_neg = np.zeros((100, 100))

    for i in range(100):
        for j in range(100):
            pos, neg = torque_func(np.ones(nbq) * q[i], np.ones(nbq) * qdot[j])
            tau_pos[i, j] = pos.__array__()[dof]
            tau_neg[i, j] = neg.__array__()[dof]

    q = q * 180 / pi
    qdot = qdot * 180 / pi

    fig_pos = plt.figure()
    fig_neg = plt.figure()

    ax_neg = fig_neg.gca(projection='3d')
    ax_pos = fig_pos.gca(projection='3d')

    q, qdot = np.meshgrid(qdot, q)

    ax_pos.plot_surface(q, qdot, tau_pos)
    ax_neg.plot_surface(q, qdot, tau_neg)

    ax_pos.set_xlabel('Qdot', fontsize=15)
    ax_pos.set_ylabel('Q', fontsize=15)
    ax_pos.set_zlabel('Tau', fontsize=15)

    ax_neg.set_xlabel('Qdot', fontsize=15)
    ax_neg.set_ylabel('Q', fontsize=15)
    ax_neg.set_zlabel('Tau', fontsize=15)

    plt.show()
