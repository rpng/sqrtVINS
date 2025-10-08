import matplotlib.pyplot as plt
import numpy as np
import sys

if len(sys.argv) != 2:
    print("Usage: python plot_script.py <data_file_path>")
    sys.exit(1)

data_file_path = sys.argv[1]

try:
    # Load data from the text file
    data = np.loadtxt(data_file_path, delimiter=",")

    # Extract timestamps and condition numbers
    timestamps = data[:, 0]
    condition_numbers = data[:, 1]

    # Calculate relative timestamps (subtract the first timestamp)
    relative_timestamps = timestamps - timestamps[0]

    # Filter data where condition number is less than or equal to 1,000,000
    condition_numbers_filtered = condition_numbers[condition_numbers <= 5e6]
    relative_timestamps_filtered = relative_timestamps[
        : len(condition_numbers_filtered)
    ]

    # Plotting
    plt.figure(figsize=(10, 6))
    plt.plot(
        relative_timestamps_filtered,
        condition_numbers_filtered * condition_numbers_filtered,
        # marker="o",
        linestyle="-",
        color="b",
    )
    plt.xlabel("Relative Timestamp")
    plt.ylabel("Condition Number")
    plt.title(
        "Plot of Relative Timestamp vs Condition Number (Condition Number <= 1,000,000)"
    )
    plt.grid()
    plt.show()

except IOError:
    print(f"Error: Could not read the file '{data_file_path}'")
except Exception as e:
    print(f"An error occurred: {e}")
