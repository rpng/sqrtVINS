import numpy as np
import matplotlib.pyplot as plt
import argparse
from matplotlib.colors import ListedColormap

# Create a command-line argument parser
parser = argparse.ArgumentParser(description="Plot a binary matrix from a text file")
parser.add_argument(
    "filename", help="Path to the text file containing the binary matrix"
)

# Parse the command-line arguments
args = parser.parse_args()

# Read the specified text file and convert it into a NumPy array
with open(args.filename, "r") as file:
    lines = file.readlines()
    matrix_data = [list(map(int, line.strip().split())) for line in lines]

n = 0
matrix = np.array(matrix_data)
matrix[:n][matrix[:n] == 1] = 2


# Set the figure size to make the bounding box larger (adjust the numbers as needed)
plt.figure(figsize=(3, 5))  # Width: 8 inches, Height: 6 inches

# Define a colormap with white, gray, and violet colors
cmap = plt.cm.colors.ListedColormap(["white", "#89CFF0", "violet"])

# Display the matrix as an image using the defined colormap
plt.imshow(matrix, cmap=cmap, interpolation="nearest")

# Set the color limits for the colormap
plt.clim(0, 2)

# Add a colorbar for reference
# cbar = plt.colorbar(ticks=[0, 1, 2])
# cbar.set_ticklabels(["White", "Gray", "Violet"])

# Remove x and y axis numbers and ticks
plt.xticks([])
plt.yticks([])

# Make the x and y axis thicker
ax = plt.gca()
ax.spines["top"].set_linewidth(2)  # Top border
ax.spines["right"].set_linewidth(2)  # Right border
ax.spines["bottom"].set_linewidth(2)  # Bottom border
ax.spines["left"].set_linewidth(2)  # Left border

# Show the plot
plt.tight_layout()
plt.show()
