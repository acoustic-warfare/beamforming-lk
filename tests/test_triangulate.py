import numpy as np
from numpy import cross, dot
import matplotlib.pyplot as plt
import math

# Initialize empty lists to collect data
origin_1 = np.array([2.6, 0, 0])
direction_1 = []

origin_2 = np.array([-2.6, 0, 0])
direction_2 = []

# Open and read the file
with open("tests/Targets.txt", "r") as file:
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

print(len(direction_1))

# Convert lists to numpy arrays
direction_1 = np.array(direction_1)
direction_2 = np.array(direction_2)


def triangulatePoints(r1, e1, r2, e2):
    n = cross(e1, e2)
    # print("n", n)

    t1 = dot(cross(e2, n), (r2 - r1)) / dot(n, n)
    t2 = dot(cross(e1, n), (r2 - r1)) / dot(n, n)
    # print("t", t1, t2)

    p1 = r1 + t1 * e1
    p2 = r2 + t2 * e2

    return p1, p2


target_list = []
history_target_list = []


def track(p):
    # check if target are close to any old targets
    new_track = True
    for i, (track_point, time_since_last_hit, total_hits) in enumerate(target_list):
        track_distance = logarithmic_value(total_hits, 1, 2)
        # print("track_distance", track_distance)
        if (
            np.abs(track_point[0] - p[0]) < track_distance
            and np.abs(track_point[1] - p[1]) < track_distance
            and np.abs(track_point[2] - p[2]) < track_distance
        ):
            target_list[i] = (p, 0, total_hits + 1)
            new_track = False
        else:
            target_list[i] = (track_point, time_since_last_hit + 1, total_hits)

    # adds new target (no match found)
    if new_track:
        target_list.append((p, 0, 1))

    # find old stuff to remove
    for i, (track_point, time_since_last_hit, total_hits) in enumerate(target_list):
        if time_since_last_hit > 20:
            target_list[i] = (track_point, time_since_last_hit, 0)

    # go throung and print some data
    for i, (track_point, time_since_last_hit, total_hits) in enumerate(target_list):
        if total_hits > 0:
            print(
                "track",
                i,
                ", time_since_last_hit",
                time_since_last_hit,
                ", total_hits",
                total_hits,
                ", track_distance",
                logarithmic_value(total_hits, 1, 2),
            )

    # find best hit to return
    best_total_hits = 0
    best_time_since_last_hit = 99
    best_point = np.zeros(3)
    best_track_id = 99
    for i, (track_point, time_since_last_hit, total_hits) in enumerate(target_list):
        if total_hits > best_total_hits:
            best_total_hits = total_hits
            best_point = track_point
            best_time_since_last_hit = time_since_last_hit
            best_track_id = i
    print(
        "BEST_track",
        best_track_id,
        ", total_hits",
        best_total_hits,
        ", time_since_last_hit",
        best_time_since_last_hit,
        "\n",
    )
    history_target_list.append([*target_list])
    return best_point, best_track_id


def logarithmic_value(total_hits, min_value, max_value):
    if total_hits < 1:
        return 0

    log_min = min_value
    log_max = max_value
    log_range = log_max - log_min

    log_value = math.log2(total_hits)
    log_value = (log_value - math.log2(1)) / (
        math.log2(20) - math.log2(1)
    )  # Normalize logarithm to [0, 1]

    result = log_min + log_range * log_value
    return result


def plot_direction_vectors(ax, origin_1, direction_1, origin_2, direction_2):
    line_space = np.linspace(0, 15, 100)  # Range for the line
    line_points_1 = origin_1 + line_space[:, np.newaxis] * direction_1[i]
    line_points_2 = origin_2 + line_space[:, np.newaxis] * direction_2[i]
    ax.plot(
        line_points_1[:, 0],
        line_points_1[:, 1],
        line_points_1[:, 2],
        color="b",
        linewidth=0.5,
    )
    ax.plot(
        line_points_2[:, 0],
        line_points_2[:, 1],
        line_points_2[:, 2],
        color="y",
        linewidth=0.5,
    )


def plot_intersections(ax, p, p1, p2):
    ax.scatter(p1[0], p1[1], p1[2], color="b", s=3)
    ax.scatter(p2[0], p2[1], p2[2], color="y", s=3)
    ax.scatter(p[0], p[1], p[2], color="g", s=3)


def plot_best_target(ax, colors, expected_nr_targets, best_target, best_track_id):
    color = colors[best_track_id % expected_nr_targets]
    target = ax.scatter(
        best_target[0], best_target[1], best_target[2], color=color, s=50
    )
    plt.pause(0.005)
    target.remove()


def plot_all_targets(ax, colors, expected_nr_targets, target_list):
    targets = []
    for i, (track_point, time_since_last_hit, total_hits) in enumerate(target_list):
        if total_hits > 0:
            color = colors[i % expected_nr_targets]
            targets.append(
                ax.scatter(
                    track_point[0],
                    track_point[1],
                    track_point[2],
                    color=color,
                    s=10 + total_hits / 2,
                )
            )
    plt.pause(0.005)
    for target in targets:
        target.remove()


target_point = np.zeros(3)
failed = 0

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
plt.axis("equal")

expected_nr_targets = 60
tab20 = plt.get_cmap("tab20")
colors = [tab20(i % 20) for i in range(expected_nr_targets)]

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

ax.set_xlim([-6, 6])
ax.set_ylim([-6, 6])
ax.set_zlim([-1, 15])

origin = np.array([0, 0, 0])
ax.scatter(
    origin[0],
    origin[1],
    origin[2],
    color="r",
    s=100,
)
ax.scatter(
    origin_1[0],
    origin_1[1],
    origin_1[2],
    color="g",
    s=100,
)
ax.scatter(
    origin_2[0],
    origin_2[1],
    origin_2[2],
    color="g",
    s=100,
)

for i in range(0, len(direction_1)):
    p1, p2 = triangulatePoints(origin_1, direction_1[i], origin_2, direction_2[i])
    p = (p1 + p2) / 2

    if np.linalg.norm(p1 - p2) > 0.5 or p[2] < 0 or np.linalg.norm(p) > 20:
        p = np.zeros(3)

    # print(target_point, i)
    if p.all() == 0:
        failed += 1
    else:

        # plot_direction_vectors(ax, origin_1, direction_1, origin_2, direction_2)
        # plot_intersections(ax, p, p1, p2)

        best_target, best_track_id = track(p)
        # plot_best_target(ax, colors, expected_nr_targets, best_target, best_track_id)
        plot_all_targets(ax, colors, expected_nr_targets, target_list)

print("failed", failed)
print("nr_of_tracks", len(target_list))

for i, (track_point, time_since_last_hit, total_hits) in enumerate(target_list):
    color = colors[i % expected_nr_targets]
    ax.scatter(track_point[0], track_point[1], track_point[2], color=color, s=10)

plt.show()

# plot log distance function
i_values = np.linspace(1, 100, 100)  # Range for the line
track_distances = [logarithmic_value(i, 1, 2) for i in i_values]
plt.figure(figsize=(10, 6))
plt.plot(i_values, track_distances, linestyle="-", color="b")
plt.xlabel("Number of Hits")
plt.ylabel("Track Distance")
plt.grid(True)
plt.show()

# plot history target list
# print("history_target_list", history_target_list)
# print("history_target_list length", len(history_target_list))
target_hits_over_time = {}
# Process each snapshot in history_target_list
for time_index, snapshot in enumerate(history_target_list):
    for target_id, (track_point, time_since_last_hit, total_hits) in enumerate(
        snapshot
    ):
        if target_id not in target_hits_over_time:
            target_hits_over_time[target_id] = {"time": [], "total_hits": []}

        target_hits_over_time[target_id]["time"].append(time_index)
        target_hits_over_time[target_id]["total_hits"].append(total_hits)

# Plotting history_target_list
plt.figure(figsize=(12, 8))

for target_id, data in target_hits_over_time.items():
    color = colors[target_id % expected_nr_targets]

    plt.plot(
        data["time"],
        data["total_hits"],
        color=color,
        marker="o",
        label=f"Target {target_id}",
    )

plt.xlabel("Time (Snapshot Index)")
plt.ylabel("Total Hits")
plt.grid(True)
plt.show()
