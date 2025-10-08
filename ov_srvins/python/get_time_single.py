import sys


def calculate_average_difference(filename):
    total_sum = 0
    count = 0

    with open(filename, "r") as file:
        next(file)  # Skip the title row
        for line in file:
            parts = line.split()
            total = float(parts[7])
            tracking = float(parts[1])
            total_sum += total - tracking
            count += 1

    if count > 0:
        average_difference = total_sum / count
        return average_difference
    else:
        return 0.0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py filename")
    else:
        filename = sys.argv[1]
        average_difference = calculate_average_difference(filename)
        print(f"Average Total-Tracking: {average_difference:.2f}")
