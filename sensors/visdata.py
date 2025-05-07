import sys
import json
import matplotlib.pyplot as plt

def process_file(file_path):
    timestamps = []
    headings = []
    pitches = []
    rolls = []

    sensor_locs = []
    joint_name = ""

    with open(file_path,"r") as file:
        data = json.loads(file.readline())
        for sensor in data["sensors"]:
                sensor_locs.append(sensor["location"])

    for i,loc in enumerate(sensor_locs):
        print(f"({i+1}) {loc}")

    inp = input("which sensor would you like to plot: ").strip()
    try:
        joint_name = sensor_locs[int(inp)-1]
    except Exception as e:
        print(f"Error processing input: {e}")
        sys.exit(1)


    with open(file_path,"r") as file:
        for line in file:
            data = json.loads(line)
            timestamps.append(data["timestamp"])
           
            for sensor in data["sensors"]:
                if sensor["location"] == joint_name:
                    headings.append(sensor["euler"]["heading"])
                    pitches.append(sensor["euler"]["pitch"])
                    rolls.append(sensor["euler"]["roll"])
                    break
    
    # Convert timestamps to indices for better readability on the plot
    time_indices = range(len(timestamps))

    # Plot data
    plt.figure(figsize=(10, 5))
    plt.plot(time_indices, headings, label="Heading", color="blue")
    plt.plot(time_indices, pitches, label="Pitch", color="red")
    plt.plot(time_indices, rolls, label="Roll", color="green")

    # Labels and title
    plt.xlabel("Time (Index)")
    plt.ylabel("Euler Angles (Degrees)")
    plt.title(f"IMU Data for {joint_name}")
    plt.legend()
    plt.grid()

    # Show plot
    plt.show()




def main():
    if len(sys.argv) < 2:
        print("Usage: python script.py <file_path>",file=sys.stderr)
        sys.exit(1)

    file_path = sys.argv[1]
    process_file(file_path)

if __name__ == "__main__":
    main()