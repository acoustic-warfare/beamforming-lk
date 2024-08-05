import numpy as np
from numpy import cross, dot
import matplotlib.pyplot as plt
import math

## MAGIC NUMBERS START##
intersection_grace_radius = 0.2  # meters between two vectors to find a intersect

plt_pause_on = True  # True/False = 3d plot/no 3d plot, no 3d plot makes the program run faster
plt_pause_factor = 0.1  # make the pauses shorter
## MAGIC NUMBERS END ##

# Initialize empty lists to read data
origin_1 = np.array([1, 0, 0])
direction_1 = []
origin_2 = np.array([-1, 0, 0])
direction_2 = []
timestamps = []


def read_file(direction_1, direction_2, timestamps, file_name):
    with open(file_name, "r") as file:
        for line in file:
            parts = line.split(";")
            first_half = parts[0].split(",")
            second_half = parts[1].split(",")

            first_half_origin_1 = first_half[0].split()
            first_half_direction_1 = first_half[1].split()

            second_half_origin_1 = second_half[0].split()
            second_half_direction_1 = second_half[1].split()

            # Extract data for the first half
            direction_1.append(
                [
                    float(first_half_direction_1[0]),
                    float(first_half_direction_1[1]),
                    float(first_half_direction_1[2]),
                ]
            )

            # Extract data for the second half
            direction_2.append(
                [
                    float(second_half_direction_1[0]),
                    float(second_half_direction_1[1]),
                    float(second_half_direction_1[2]),
                ]
            )
            timestamps.append(int(parts[2].strip()))

    # print(len(direction_1))

    # Convert lists to numpy arrays
    direction_1 = np.array(direction_1)
    direction_2 = np.array(direction_2)
    timestamps = np.array(timestamps)
    return direction_1, direction_2, timestamps


direction_1, direction_2, timestamps = read_file(direction_1, direction_2, timestamps, "math_toolbox/recorderd_data/intersections_demo_plot.txt")


def triangulatePoints(r1, e1, r2, e2):
    n = cross(e1, e2)

    t1 = dot(cross(e2, n), (r2 - r1)) / dot(n, n)
    t2 = dot(cross(e1, n), (r2 - r1)) / dot(n, n)

    p1 = r1 + t1 * e1
    p2 = r2 + t2 * e2

    return p1, p2


def plot_direction_vector(ax, origin_1, direction_1_i, color):
    line_space = np.linspace(0, 15, 100)  # Range for the line
    line_points_1 = origin_1 + line_space[:, np.newaxis] * direction_1_i
    vector = ax.plot(
        line_points_1[:, 0],
        line_points_1[:, 1],
        line_points_1[:, 2],
        color=color,
        linewidth=0.5,
    )
    return vector[0]  # Return the first Line2D object


def plot_intersections(ax, p, p1, p2):
    # ax.scatter(p1[0], p1[1], p1[2], color="b", s=3)
    # ax.scatter(p2[0], p2[1], p2[2], color="y", s=3)
    intersection = ax.scatter(p[0], p[1], p[2], color="g", s=15)
    return intersection


failed_intersect_count = 0

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
plt.ion()  # Turn on interactive mode

max_nr_colors = 20
tab20 = plt.get_cmap("tab20")
colors = [tab20(i % 20) for i in range(20)]


def setup_3d_plot(ax):
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    ax.set_xlim([-6, 6])
    ax.set_ylim([-6, 6])
    ax.set_zlim([-1, 15])
    ax.view_init(elev=-30, azim=91, roll=0)

    origin = np.array([0, 0, 0])
    ax.scatter(origin[0], origin[1], origin[2], color="r", s=30)
    ax.scatter(origin_1[0], origin_1[1], origin_1[2], color="y", s=100)
    ax.scatter(origin_2[0], origin_2[1], origin_2[2], color="b", s=100)


setup_3d_plot(ax)

plt_intersection = 0
plt_pause = 0

vector_1 = []
vector_2 = []
intersections = []

last_hit_time = timestamps[0]
current_time = timestamps[0]


for i in range(3000, len(direction_1)):
    p1, p2 = triangulatePoints(origin_1, direction_1[i], origin_2, direction_2[i])
    p = (p1 + p2) / 2

    current_time = timestamps[i]
    if np.linalg.norm(p1 - p2) > intersection_grace_radius or p[2] < 0 or np.linalg.norm(p) > 20 or p[2] < 2:
        p = np.zeros(3)

    if p.all() == 0:
        failed_intersect_count += 1

    else:
        plt_intersection += 1
        intersections.append(plot_intersections(ax, p, p1, p2))

    vector_1.append(plot_direction_vector(ax, origin_1, direction_1[i], color="y"))
    vector_2.append(plot_direction_vector(ax, origin_2, direction_2[i], color="b"))
    plt.draw()
    plt_pause += 1

    if min(len(vector_1), len(vector_2)) > 5:

        if plt_pause_on:
            # print((current_time - last_hit_time) * 10**-9)
            plt.pause(
                (current_time - last_hit_time) * 10**-9 * plt_pause_on * plt_pause_factor + 0.000000001
            )  # Pause for a short time to update the plot
            last_hit_time = current_time
        else:
            if plt_intersection > 0:
                plt.pause(0.5)
            else:
                plt.pause(0.2)

        plt_pause = 0
        plt_intersection = 0
        for vector in vector_1:
            vector.remove()
        for vector in vector_2:
            vector.remove()
        for intersection in intersections:
            intersection.remove()

        vector_1 = []
        vector_2 = []
        intersections = []


plt.ioff()  # Turn off interactive mode
plt.close()
print("Done!")
