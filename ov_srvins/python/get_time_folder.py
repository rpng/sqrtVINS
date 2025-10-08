import os
import sys


def calculate_average_difference(filename):
    total_sum = 0
    count = 0

    with open(filename, "r") as file:
        next(file)  # Skip the title row
        for line in file:
            parts = line.split(",")
            total = float(parts[7])
            tracking = float(parts[1])
            total_sum += total - tracking
            count += 1

    if count > 0:
        average_difference = total_sum / count
        return average_difference * 1000
    else:
        return 0.0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py folder_path")
    else:
        folder_path = sys.argv[1]
        if not os.path.isdir(folder_path):
            print("Invalid folder path")
        else:
            txt_files = [
                f for f in os.listdir(folder_path) if f.lower().endswith(".txt")
            ]
            if not txt_files:
                print("No txt files found in the folder")
            else:
                total_average = 0
                for txt_file in txt_files:
                    txt_path = os.path.join(folder_path, txt_file)
                    average_difference = calculate_average_difference(txt_path)
                    total_average += average_difference
                    print(
                        f"Average Total-Tracking for {txt_file}: {average_difference:.2f}"
                    )

                overall_average = total_average / len(txt_files)
                print(f"Overall Average Total-Tracking: {overall_average:.2f}")
