import numpy as np
from numpy import cross, dot
import matplotlib.pyplot as plt
import math

## MAGIC NUMBERS START##
intersection_grace_radius = 1  # meters between two vectors to find a intersect

min_track_distance = 1
max_track_distance = 2
track_distance_factor = 1  # factor to change the kill_time

min_track_kill_time = 1  # min numger of second this is a baseline for new targets with very few hits
max_track_kill_time = 2  # max number of seconds the kill_time can acheve for a target with many hits
kill_time_factor = 1  # factor to change the kill_time

plt_pause_on = False
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


direction_1, direction_2, timestamps = read_file(direction_1, direction_2, timestamps, "tests/Targets2.txt")


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


def track(p, timestamp, same_hit_count, unique_hit_count):
    # check if target are close to any old targets
    new_track = True
    same_track = False
    current_time = timestamp
    for i, (track_point, time_since_last_hit, total_hits, timestamp) in enumerate(target_list):
        track_distance = track_distance_generation(total_hits, min_track_distance, max_track_distance, track_distance_factor)
        # print("track_distance", track_distance)
        if (
            np.abs(track_point[0] - p[0]) < track_distance
            and np.abs(track_point[1] - p[1]) < track_distance
            and np.abs(track_point[2] - p[2]) < track_distance
        ):
            if track_point[0] == p[0] and track_point[1] == p[1] and track_point[2] == p[2]:
                # same exact point
                same_track = True
                new_track = False
                same_hit_count = same_hit_count + 1
                break
            else:
                target_list[i] = (p, 0, total_hits + 1, current_time)
                new_track = False
                # print("old target")
                break

    # adds new target (no match found)
    if new_track:
        target_list.append((p, 0, 1, current_time))
        # print("new target")

    # add +1 to all time_since_last_hit (make them older) (this includes current hit)
    if same_track == False:
        unique_hit_count = unique_hit_count + 1
        for i, (track_point, time_since_last_hit, total_hits, timestamp) in enumerate(target_list):
            target_list[i] = (track_point, time_since_last_hit + 1, total_hits, timestamp)

        # find old tracks (with time greate than kill_time) to remove
        for i, (track_point, time_since_last_hit, total_hits, timestamp) in enumerate(target_list):
            if (
                int(current_time) - int(timestamp)
                > kill_time_generation(total_hits, min_track_kill_time, max_track_kill_time, kill_time_factor) * 10**9
            ):
                target_list[i] = (track_point, time_since_last_hit, 0, timestamp)

    # find best hit to return
    best_total_hits = 0
    best_point = np.zeros(3)
    best_track_id = 99
    for i, (track_point, time_since_last_hit, total_hits, timestamp) in enumerate(target_list):
        if total_hits > best_total_hits:
            best_total_hits = total_hits
            best_point = track_point
            best_track_id = i

    history_target_list.append([[*target_list], current_time])
    return best_point, best_track_id, same_hit_count, unique_hit_count


def track_distance_generation(total_hits, min_value, max_value, track_distance_factor):
    if total_hits < 1:
        return 0  # stop killed points from gathering more targets

    # y=0.325 ln(x+1)+0.5
    result = 0.325 * math.log(total_hits + 1) + 0.5
    if result < min_value:
        result = min_value
    elif result > max_value:
        result = max_value

    return result * track_distance_factor


def kill_time_generation(total_hits, min_value, max_value, kill_time_factor):
    # y=ln(x+1)+0.5
    result = math.log(total_hits + 1) + 0.5
    if result < min_value:
        result = min_value
    elif result > max_value:
        result = max_value

    return result * kill_time_factor


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


def plot_best_target(ax, colors, max_nr_colors, best_target, best_track_id):
    color = colors[best_track_id % max_nr_colors]

    targets = []
    targets.append(ax.scatter(best_target[0], best_target[1], best_target[2], color=color, s=75, marker="x"))
    return targets


def plot_all_targets(ax, colors, max_nr_colors, target_list):
    targets = []
    for i, (track_point, time_since_last_hit, total_hits, timestamp) in enumerate(target_list):
        if total_hits > 0:
            color = colors[i % max_nr_colors]
            targets.append(
                ax.scatter(
                    track_point[0],
                    track_point[1],
                    track_point[2],
                    color=color,
                    s=10 + total_hits / 2,
                )
            )
    return targets


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
    ax.scatter(origin[0], origin[1], origin[2], color="r", s=100)
    ax.scatter(origin_1[0], origin_1[1], origin_1[2], color="g", s=100)
    ax.scatter(origin_2[0], origin_2[1], origin_2[2], color="g", s=100)


setup_3d_plot(ax)

same_hit_count = 0
unique_hit_count = 0

best_target = []
all_targets = []

last_hit_time = timestamps[0]
current_time = timestamps[0]

for i in range(0, len(direction_1)):
    p1, p2 = triangulatePoints(origin_1, direction_1[i], origin_2, direction_2[i])
    p = (p1 + p2) / 2

    current_time = timestamps[i]
    if np.linalg.norm(p1 - p2) > intersection_grace_radius or p[2] < 0 or np.linalg.norm(p) > 20:
        p = np.zeros(3)

    if p.all() == 0:
        failed_intersect_count += 1
    else:
        if plt_pause_on:
            print((current_time - last_hit_time) * 10**-9)
            plt.pause(
                (current_time - last_hit_time) * 10**-9 * plt_pause_on * plt_pause_factor
            )  # Pause for a short time to update the plot
        last_hit_time = current_time

        # plot_direction_vectors(ax, origin_1, direction_1, origin_2, direction_2)
        # plot_intersections(ax, p, p1, p2)

        for target in all_targets:
            target.remove()

        for target in best_target:
            target.remove()

        best_target_point, best_track_id, same_hit_count, unique_hit_count = track(p, timestamps[i], same_hit_count, unique_hit_count)
        best_target = plot_best_target(ax, colors, max_nr_colors, best_target_point, best_track_id)
        all_targets = plot_all_targets(ax, colors, max_nr_colors, target_list)

        plt.draw()


plt.ioff()  # Turn off interactive mode
plt.close()
print("Done!")


fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
setup_3d_plot(ax)
ax.set_box_aspect([1, 1, 1])  # This ensures equal aspect ratio
for i, (track_point, time_since_last_hit, total_hits, timestamp) in enumerate(target_list):
    color = colors[i % max_nr_colors]
    ax.scatter(track_point[0], track_point[1], track_point[2], color=color, s=10)
plt.show()


# plot log distance function
i_values = np.linspace(1, 100, 100)  # Range for the line
track_distances = [track_distance_generation(i, min_track_distance, max_track_distance, track_distance_factor) for i in i_values]
kill_times = [kill_time_generation(i, min_track_kill_time, max_track_kill_time, kill_time_factor) for i in i_values]
plt.figure(figsize=(10, 6))
plt.subplot(2, 1, 1)
plt.plot(i_values, track_distances, linestyle="-", color="b")
plt.ylabel("Track Distance")
plt.subplot(2, 1, 2)
plt.plot(i_values, kill_times, linestyle="-", color="r")
plt.xlabel("Number of Hits")
plt.ylabel("Kill Time")
plt.tight_layout()
plt.show()

# plot history target list
target_hits_over_time = {}
plt.figure(figsize=(12, 8))

# Process each snapshot in history_target_list
for snapshot, time_of_snapshot in history_target_list:
    for target_id, (track_point, time_since_last_hit, total_hits, timestamp) in enumerate(snapshot):
        if target_id not in target_hits_over_time:
            target_hits_over_time[target_id] = {"time": [], "total_hits": []}

        target_hits_over_time[target_id]["time"].append((time_of_snapshot - timestamps[0]) * 10**-9)
        target_hits_over_time[target_id]["total_hits"].append(total_hits)

for target_id, data in target_hits_over_time.items():
    color = colors[target_id % max_nr_colors]

    # Separate data based on the total_hits condition
    times_with_hits = [t for t, hits in zip(data["time"], data["total_hits"]) if hits >= 1]
    hits_with_hits = [hits for hits in data["total_hits"] if hits >= 1]

    times_without_hits = [t for t, hits in zip(data["time"], data["total_hits"]) if hits < 1]
    hits_without_hits = [hits for hits in data["total_hits"] if hits < 1]

    # Plot points with hits >= 1 with markers
    if times_with_hits:
        plt.plot(times_with_hits, hits_with_hits, color=color, marker="o", label=f"Target {target_id}")

plt.xlabel("Time (Snapshot Index)")
plt.ylabel("Total Hits")
plt.grid(True)
plt.show()

# Print some stats
print("failed_intersect_count", failed_intersect_count)
print("unique_hit_count", unique_hit_count)
print("same_hit_count", same_hit_count)
print("nr_of_tracks", len(target_list))
