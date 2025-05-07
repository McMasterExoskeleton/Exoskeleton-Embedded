import sys
import json
import matplotlib.pyplot as plt
from collections import defaultdict
from datetime import datetime

def main():
    if len(sys.argv) != 2:
        print(f"Usage: python {sys.argv[0]} <path_to_file>")
        sys.exit(1)

    file_path = sys.argv[1]

    # Dictionary to store only the roll data (plus timestamps) for each sensor
    sensor_data = defaultdict(lambda: {"timestamp": [], "roll": []})

    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if not line:
                continue
            record = json.loads(line)

            timestamp = datetime.strptime(record['timestamp'], "%Y-%m-%d %H:%M:%S.%f")

            for sensor in record['sensors']:
                loc = sensor['location']
                sensor_data[loc]['timestamp'].append(timestamp)
                sensor_data[loc]['roll'].append(sensor['euler']['roll'])

    # Adjust this order if needed
    sensor_order = [
        "Left Knee", "Left Ankle", 
        "Right Ankle", "Right Hip", 
        "Right Knee", "Left Hip"
    ]

    # Subsampling factor: plot every 10th point, for example
    subsample_factor = 10

    plt.figure(figsize=(20, 6))

    for loc in sensor_order:
        # Take every 10th point
        timestamps = sensor_data[loc]['timestamp'][::subsample_factor]
        rolls = sensor_data[loc]['roll'][::subsample_factor]
        plt.plot(timestamps, rolls, label=loc)

    plt.title("Roll Data Over Time (Subsampled)")
    plt.xlabel("Time")
    plt.ylabel("Roll (degrees)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
