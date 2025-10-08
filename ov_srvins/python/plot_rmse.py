import matplotlib.pyplot as plt
import numpy as np
import sys
import os
from matplotlib.lines import Line2D

if len(sys.argv) != 2:
    print("Usage: python plot_script.py <folder_name>")
    sys.exit(1)

folder_name = sys.argv[1]
folder_path = os.path.join(os.getcwd(), folder_name)

# Load the timestamps from the first file
first_file_path = os.path.join(folder_path, "ov_double.txt")
timestamps = np.loadtxt(first_file_path, delimiter=",")[:, 0]

# Function to load and plot data for a given file and group
def plot_data(file_name, group_name):
    file_path = os.path.join(folder_path, file_name)

    # Load data from the text file
    data = np.loadtxt(file_path, delimiter=",")

    # Extract errors
    position_errors = data[:, 1]
    orientation_errors = data[:, 2]

    # Calculate relative timestamps using the timestamps from the first file
    relative_timestamps = data[:, 0] - timestamps[0]

    return relative_timestamps, position_errors, orientation_errors


# Function to load condition numbers
def load_condition_numbers():
    cond_file_path = os.path.join(folder_path, "cond.txt")
    cond_data = np.loadtxt(cond_file_path, delimiter=",")
    relative_timestamps_cond = (
        cond_data[:, 0] - timestamps[0]
    )  # Relative timestamps for condition numbers
    condition_numbers = cond_data[:, 1]  # Extract condition numbers

    # Filter condition numbers larger than 3e5
    mask = condition_numbers <= 3e5
    return (
        relative_timestamps_cond[mask],
        condition_numbers[mask],
    )  # Return filtered data


# Function to load condition numbers
def load_condition_numbers_F():
    cond_file_path = os.path.join(folder_path, "cond_F.txt")
    cond_data = np.loadtxt(cond_file_path, delimiter=",")
    relative_timestamps_cond = (
        cond_data[:, 0] - timestamps[0]
    )  # Relative timestamps for condition numbers
    condition_numbers = cond_data[:, 1]  # Extract condition numbers

    # Filter condition numbers larger than 3e5
    mask = condition_numbers <= 3e5
    return (
        relative_timestamps_cond[mask],
        condition_numbers[mask],
    )  # Return filtered data


# List of file names and group names
file_names = [
    "ov_double.txt",
    "ov_float.txt",
    "srf_double.txt",
    "srf_float.txt",
    "srif_double.txt",
    "srif_float.txt",
]

group_names = [
    "EKF(d)",
    "EKF(f)",
    "SRF(d)",
    "SRF(f)",
    "SRIF(d)",
    "SRIF(f)",
]

# Define line styles for double and float
line_styles = ["-", "--", "-", "--", "-", "--"]

# Define colors for each group (ov_double, ov_float, srf_double, srf_float, srif_double, srif_float)
colors = ["b", "b", "m", "m", "g", "g"]

# Plot data for orientation errors
plt.figure(figsize=(10, 8))
plt.subplot(4, 1, 1)
plt.title("Orientation Error")
plt.grid()
plt.xlim(0, 1750)

legend_handles = []

for file_name, group_name, line_style, color in zip(
    file_names, group_names, line_styles, colors
):
    relative_timestamps, _, orientation_errors = plot_data(file_name, group_name)
    (line,) = plt.plot(
        relative_timestamps,
        orientation_errors,
        linestyle=line_style,
        color=color,
        alpha=0.7,
    )
    if line_style == "--":
        line.set_dashes([3, 3])  # Make the dash line sparser
    legend_handles.append(
        Line2D([0], [0], color=color, linestyle=line_style, linewidth=2)
    )

plt.ylabel("Ori. Err. (deg)")
plt.legend(
    legend_handles,
    group_names,
    loc="upper center",
    bbox_to_anchor=(0.5, 2.5),
    fancybox=True,
    shadow=True,
    ncol=3,
)

# Plot data for position errors
plt.subplot(4, 1, 2)
plt.title("Position Error")
plt.grid()
plt.xlim(0, 1750)

legend_handles = []

for file_name, group_name, line_style, color in zip(
    file_names, group_names, line_styles, colors
):
    relative_timestamps, position_errors, _ = plot_data(file_name, group_name)
    (line,) = plt.plot(
        relative_timestamps,
        position_errors,
        linestyle=line_style,
        color=color,
        alpha=0.7,
    )
    if line_style == "--":
        line.set_dashes([3, 3])  # Make the dash line sparser
    legend_handles.append(
        Line2D([0], [0], color=color, linestyle=line_style, linewidth=2)
    )

plt.ylabel("Pos. Err. (m)")
# plt.legend(legend_handles, group_names)

# Plot data for condition numbers
plt.subplot(4, 1, 3)
plt.title("Condition Number of R")
plt.grid()

relative_timestamps_cond, condition_numbers = load_condition_numbers()

plt.plot(relative_timestamps_cond, condition_numbers, color="k", linestyle="-")
# plt.xlabel("Time")
plt.ylabel("$\kappa(R)$")
plt.ticklabel_format(axis="y", style="sci", scilimits=(0, 0))
plt.xlim(0, 1750)

# Plot data for S condition numbers
plt.subplot(4, 1, 4)
plt.title("Condition Number of F")
plt.grid()

relative_timestamps_cond_F, condition_numbers_F = load_condition_numbers_F()

plt.plot(relative_timestamps_cond_F, condition_numbers_F, color="k", linestyle="-")
plt.xlabel("Time(s)")
plt.ylabel("$\kappa(F)$")

# Set x-axis limits to start from 0 and end at 1750
plt.xlim(0, 1750)

# Use scientific notation for the y-axis of the "Condition Number of R" plot
# plt.ticklabel_format(axis="y", style="sci", scilimits=(0, 0))

# Configure and display the plots
plt.tight_layout()
plt.show()
