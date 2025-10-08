import argparse
import os
import numpy as np
import matplotlib.pyplot as plt

# Create a command-line argument parser
parser = argparse.ArgumentParser(
    description="Generate separate curve plots for position error data in subfolders."
)
parser.add_argument(
    "folder",
    type=str,
    help="Path to the folder containing the subfolders with data files",
)
parser.add_argument(
    "--threshold",
    type=float,
    default=2.0,
    help="Position error threshold (default: 2.0)",
)
args = parser.parse_args()

# Validate that the specified folder exists
if not os.path.isdir(args.folder):
    print(f"Error: The folder '{args.folder}' does not exist.")
    exit(1)

# Initialize lists to store position errors and total data points
all_position_errors = []
total_data_points = 0

# Create a single figure and axis to plot all subfolders
plt.figure(figsize=(6, 2.5))
ax = plt.subplot(111)  # Create a single subplot

# Define line colors and transparencies
line_colors = ["b", "g", "r", "c", "m", "y", "k"]
line_alpha = 0.7

# Loop through all subfolders in the main folder
for i, subfolder in enumerate(os.listdir(args.folder)):
    subfolder_path = os.path.join(args.folder, subfolder)

    if os.path.isdir(subfolder_path):
        subfolder_position_errors = []

        # Loop through all files in the subfolder
        for filename in os.listdir(subfolder_path):
            if filename.endswith(".txt"):
                file_path = os.path.join(subfolder_path, filename)

                # Load data from the current file
                try:
                    data = np.loadtxt(file_path, delimiter=",", skiprows=1)
                except FileNotFoundError:
                    print(
                        f"Warning: Skipping file '{file_path}' as it could not be loaded."
                    )
                    continue

                # Extract position error values
                position_errors = data[:, 1]

                # Update the total data points
                total_data_points += len(position_errors)

                # Append position errors from the current file to the list
                all_position_errors.extend(position_errors)
                subfolder_position_errors.extend(position_errors)

        subfolder_position_errors.append(args.threshold)

        # Calculate the number of data points below the threshold for the subfolder
        data_points_below_threshold = np.sum(
            np.array(subfolder_position_errors) < args.threshold
        )

        # Determine the effective threshold for the subfolder
        subfolder_threshold = args.threshold

        # Calculate the percentage below the effective threshold for the subfolder
        percentage_below_threshold = (
            data_points_below_threshold / len(subfolder_position_errors)
        ) * 100

        print(
            f"Percentage of data points below {subfolder_threshold:.2f} cm in '{subfolder}': {percentage_below_threshold:.2f}%"
        )

        # Create a curve plot for the subfolder
        sorted_subfolder_position_errors = np.sort(subfolder_position_errors)
        cumulative_percentages_below_threshold = [
            np.sum(sorted_subfolder_position_errors < threshold)
            / len(sorted_subfolder_position_errors)
            * 100
            for threshold in sorted_subfolder_position_errors
        ]

        # Plot the subfolder's data on the same axis with specified color and transparency
        ax.plot(
            sorted_subfolder_position_errors * 100,
            cumulative_percentages_below_threshold,
            label=f"{subfolder}",
            # marker="o",
            color=line_colors[i % len(line_colors)],
            alpha=line_alpha,
            linewidth=3,
        )

# Customize the plot
ax.set_xlabel("Position Error Threshold (cm)")
ax.set_ylabel("Correctly Locailzed (%)")
ax.set_title("Cumulative Distribution of Position Errors")
ax.legend(loc="center right", bbox_to_anchor=(1.4, 0.5), fancybox=True)
ax.grid(True)
ax.set_xlim(0, args.threshold * 100)
ax.set_ylim(0, 105.0)

plt.tight_layout()
plt.subplots_adjust(right=0.7)


# Show the plot with all subfolders
plt.show()
