import unittest

import numpy as np

import build.lib.antenna as module


print(module.__version__)
print(module.__doc__)

# import python.antenna as truth

from build import config

DTYPE = np.float32

import matplotlib.pyplot as plt


def plot_antenna(antenna, adaptive=[], inline=False, relative=False):
    """Plot the antenna in 3D space

    Args:
        antenna (_type_): _description_
        adaptive (list, optional): _description_. Defaults to [].
        inline (bool, optional): _description_. Defaults to False.
        relative (bool, optional): _description_. Defaults to False.
    """
    # if inline:
    #     %matplotlib inline
    # else:
    #     %matplotlib qt

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")

    a = antenna.copy()
    if relative:
        a[:, 2] = a[:, 2] - a[:, 2].min()

    active = []
    inactive = []
    for i in range(a.shape[0]):
        if i in adaptive:
            active.append(i)
        else:
            inactive.append(i)

    origo = np.zeros(3)

    ax.quiver(origo[0], origo[1], origo[2], 2, 0, 0, color="r", normalize=True)
    ax.quiver(origo[0], origo[1], origo[2], 0, 2, 0, color="g", normalize=True)
    ax.quiver(origo[0], origo[1], origo[2], 0, 0, 2, color="b", normalize=True)

    ax.scatter(a[active, 2], a[active, 0], a[active, 1], c="red", marker="o")
    ax.scatter(
        a[inactive, 0],
        a[inactive, 1],
        a[inactive, 2],
        c="white",
        marker="o",
        edgecolors="black",
        linewidths=1,
    )

    # ax.axis('') #this line fits your images to screen
    plt.axis("equal")
    plt.set_cmap("jet")
    # Labels.
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    ax.set_yticklabels([])
    ax.set_xticklabels([])
    ax.set_zticklabels([])
    ax.azim = 45
    ax.dist = 10
    ax.elev = 30

    plt.show()


def generate_angles_sphere(n: int):
    """Create n angles on a unitsphere

    Args:
        n (int): number of points to create

    Returns:
        (float, float): polar angles to each point
    """
    i = np.arange(0, n, dtype=np.float32) + 0.5
    phi = np.arccos(1 - i / n)
    golden_ratio = (1 + 5**0.5) / 2

    theta = 2 * np.pi * i / golden_ratio

    return theta, phi


def generate_points_half_sphere(n: int):
    """Create n points on a dome

    Args:
        n (int): number of points

    Returns:
        np.ndarray: points -> (n, 3)
    """

    i = np.arange(0, n, dtype=np.float32) + 0.5
    phi = np.arccos(1 - i / n)
    golden_ratio = (1 + 5**0.5) / 2

    theta = 2 * np.pi * i / golden_ratio

    x = np.cos(theta) * np.sin(phi)
    y = np.sin(theta) * np.sin(phi)
    z = np.cos(phi)

    points = np.vstack((x, y, z))

    return points.T


class Antenna(unittest.TestCase):
    """Tests that antenna works as expected from the Python implementation"""

    def test_creation(self):
        """Tests that an actual Array is being created"""
        return True
        origo = np.zeros(3)
        # truth_antenna = truth.create_antenna(origo)

        module_antenna = module._create_antenna(origo)

        truth_antenna = np.zeros((config.COLUMNS * config.ROWS, 3), dtype=DTYPE)

        print(module_antenna)

        # self.assertTrue(
        #     np.allclose(module_antenna, truth_antenna, rtol=1e-05, atol=1e-08)
        # )

        if config.TEST_PLOT:
            plot_antenna(module_antenna * 10.0, relative=True)

    def test_unit_dome(self):
        n = 512
        tes = module.dome(n)

        tracker = [[0, 1, 0], [-np.sqrt(3) / 2, -1 / 2, 0], [np.sqrt(3) / 2, -1 / 2, 0]]
        tracker = np.array(tracker)

        l = 1

        r = l * np.sqrt(3) / 3

        b = 2
        d = np.linspace(0, 2 * np.pi, 100)
        y = b * np.sin(d)
        x = b * np.cos(d)

        # k = np.zeros_like(circ)
        # k = np.vstack([circ, k, k])
        # k = k.T
        # tracker /= 10
        # tes = tracker
        # p = module.dome(n)
        # tes = np.vstack([p, tes, k])
        # plot_antenna(tes)
        # plot_antenna(generate_points_half_sphere(1024))

    def test_plot(self):
        return
        import matplotlib.pyplot as plt

        # vector data. not important
        roa_vek = np.matrix([0, 0, 12])
        rob_vek = np.matrix([4, 12, 0])
        rab_vek = rob_vek - roa_vek
        f = 2000
        fab_vek = f * rab_vek / rab_vek.norm()
        mo = roa_vek.cross(fab_vek)

        ax = plt.figure().add_subplot(projection="3d")

        # if i dont set these, the plot is all zoomed in
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        origo = [0, 0, 0]
        ax.view_init(30, 45)

        # Coordinate system axis
        ax.quiver(origo[0], origo[1], origo[2], 2, 0, 0, color="r", normalize=True)
        ax.quiver(origo[0], origo[1], origo[2], 0, 2, 0, color="g", normalize=True)
        ax.quiver(origo[0], origo[1], origo[2], 0, 0, 2, color="b", normalize=True)

        # axis label placement
        ax.text(0.1, 0.0, -0.2, r"$0$")
        ax.text(1.3, 0, 0, r"$x$")
        ax.text(0, 1.3, 0, r"$y$")
        ax.text(0, 0, 1.3, r"$z$")

        # forces and moments
        ax.quiver(
            origo[0], origo[1], origo[2], mo[0], mo[1], mo[2], color="c", normalize=True
        )
        ax.text(mo[0], mo[1], mo[2], "%s" % (str("mo")), size=10, zorder=1, color="k")
        plt.show()

    # def test_sphere(self):
    #     """tests that a dome can be created"""
    #     return
    #
    #     n = 200
    #     points = truth.generate_points_half_sphere(n)
    #     truth.plot_antenna(points, relative=True)
    #
    # def test_place_antenna(self):
    #     """Tests that an antenna can be created and steered"""
    #
    #     # return
    #     antenna = truth.create_antenna()
    #
    #     shape = [[1, 0, 1], [1, 1, 1], [1, 0, 1]]
    #     merged = truth.create_combined_array(shape)
    #
    #     antenna = merged
    #     used = truth.used_sensors(antenna)
    #     used = []
    #     old = antenna.copy()
    #     antenna = truth.steer_center(antenna, 25, 45)
    #     antenna[:, 2] -= antenna[:, 2].min()
    #     print(antenna)
    #
    #     antenna = np.concatenate((old, antenna))
    #
    #     antenna *= 10
    #     truth.plot_antenna(antenna, adaptive=used, relative=True)
    #
    #     self.assertTrue(True)
    #
    # def test(self):
    #     """Tests that proper delay is being implemented"""
    #     module_delays = module.test()
    #
    #     pos = np.array([0.0, 0.0, 0.0])
    #     other = truth.create_antenna(pos)
    #     steered = truth.steer_center(other, 10.0, 335.0)
    #
    #     truth_delays = truth.compute_delays(steered)
    #     # result = np.zeros((COLUMNS, ROWS), dtype=np.float32)
    #
    #     # i = 0
    #     # for y in range(ROWS):
    #     #     for x in range(COLUMNS):
    #     #         result[y, x] = delays[i]
    #     #         i += 1
    #
    #     self.assertTrue(
    #         np.allclose(module_delays, truth_delays, rtol=1e-05, atol=1e-08)
    #     )
