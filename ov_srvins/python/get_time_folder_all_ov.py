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


def process_folder_and_subfolders(folder_path):
    total_files = 0
    total_average = 0

    for root, _, _ in os.walk(folder_path):
        txt_files = [f for f in os.listdir(root) if f.lower().endswith(".txt")]
        if txt_files:
            total_files += len(txt_files)
            subfolder_average = 0
            subfolder_files = 0

            for txt_file in txt_files:
                txt_path = os.path.join(root, txt_file)
                average_difference = calculate_average_difference(txt_path)
                subfolder_average += average_difference
                subfolder_files += 1

            if subfolder_files > 0:
                print(
                    f"Average Total-Tracking for subfolder {root}: {subfolder_average/subfolder_files:.1f}"
                )
                total_average += subfolder_average

    if total_files > 0:
        overall_average = total_average / total_files
        print(f"Overall Average Total-Tracking for all files: {overall_average:.1f}")
    else:
        print("No txt files found in the folder and its subfolders")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py folder_path")
    else:
        folder_path = sys.argv[1]
        if not os.path.exists(folder_path):
            print("Invalid folder path")
        else:
            process_folder_and_subfolders(folder_path)
