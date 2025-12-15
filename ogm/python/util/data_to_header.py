"""
Author: Teemu Mökkönen
brief: This is a helper code for generating dataset that can be used for generating c-array from the csv so that the
plain C files don't need to read the data from the disk with any open operations. This dataset should be then compiled with the project.
"""

import csv
import argparse

parser = argparse.ArgumentParser(description="Generate 2D occupancy grid from LiDAR and ground truth data")
parser.add_argument('lidar_data', type=str, help='Path to the lidar data csv file')
args = parser.parse_args()

data = []

i = 0
with open(args.lidar_data, 'r') as pcdfile:
    file = csv.reader(pcdfile)
    for cloud in file:
        if 200 < i < 700 and i % 10 == 0:
            # Parse numerical values as floats and store them as a list
            row_floats = [float(value) for value in cloud[0:]]
            data.append(row_floats)

        if i > 700:
            break

        i += 1

# Define the dimensions of the data array
num_rows = len(data)
num_cols = len(data[0])

with open('header.c', 'w') as headerfile:
    headerfile.write('#ifndef HEADER_H\n')
    headerfile.write('#define HEADER_H\n\n')
    headerfile.write(f'const float data[{num_rows}][{num_cols}] = {{\n')
    for row in data:
        # Convert list of floats to comma-separated string
        row_str = ', '.join(map(str, row))
        headerfile.write(f'    {{ {row_str} }},\n')
    headerfile.write('};\n\n')
    headerfile.write('#endif // HEADER_H\n')
