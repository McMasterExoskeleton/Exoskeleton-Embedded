"""
To get rid of sensor drift that occurs when collecting sensor value. Normalize readings on y axis. 
"""

import sys
import json

def normalize(data,section_size=100):
    if len(data) < section_size:
        section_size = len(data)

    total = sum(data[:section_size])
    translation = 0 - total/section_size
    new_data = []

    for i in range(section_size):
        new_data.append(data[i] + translation)
    
    for i in range(section_size, len(data)):
        total = total + data[i] - data[i-section_size]
        translation = -total/section_size
        new_data.append(data[i] + translation)
    
    return new_data

def fix_flip(data,diff_max = 200,section_size=100):
    new_data = [data[0]] 
    for i in range(1,len(data)):
        diff = data[i]-sum(new_data[-section_size:])/min(section_size,len(new_data))
        if diff > diff_max:
            new_data.append(data[i] - 360)
        elif diff < -diff_max:
            new_data.append(data[i] + 360)
        else:
            new_data.append(data[i])
    
    return new_data

def strip_data(data):
    while data[0] == 0:
        data.pop(0)

    while data[-1] == 0:
        data.pop()

def apply_preprocess(sensor_data,f):
    sensors = [x["location"] for x in sensor_data[0]["sensors"]]
    angle_names = [x for x in sensor_data[0]["sensors"][0]["euler"]]

    for si,sensor in enumerate(sensors):
        for angle in angle_names:
            data = [x["sensors"][si]["euler"][angle] for x in sensor_data]
            data = f(data)
            for i in range(len(sensor_data)):
                sensor_data[i]["sensors"][si]["euler"][angle] = data[i]

def process_file(file_path,output_path):
    sensor_data = []
    
    with open(file_path,"r") as file:
        for line in file:
            sensor_data.append(json.loads(line))
    
    apply_preprocess(sensor_data,fix_flip)
    apply_preprocess(sensor_data,normalize)

    print(f"\n\nsensor data: {sensor_data[0]}")
    print(f"\n\nsensor data type: {type(sensor_data[0])}\n\n")
    
    with open(output_path,"w") as file:
        for i in range(len(sensor_data)):
            file.write(json.dumps(sensor_data[i]) + "\n")


def main():
    if len(sys.argv) < 3:
        print("Usage: python script.py <file_path> <output_path>",file=sys.stderr)
        sys.exit(1)

    file_path = sys.argv[1]
    output_path = sys.argv[2]
    process_file(file_path,output_path)

if __name__ == "__main__":
    main()