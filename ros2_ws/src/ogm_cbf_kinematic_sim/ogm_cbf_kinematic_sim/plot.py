import matplotlib.pyplot as plt
import csv

# Read the data from the CSV file
file_path = "cbf_data.csv"
timestamps = []
cbf_0_values = []
cbf_1_values = []

with open(file_path, mode="r") as file:
    reader = csv.reader(file)
    next(reader)  # Skip the header
    for row in reader:
        timestamps.append(float(row[0]))
        cbf_0_values.append(float(row[1]))
        cbf_1_values.append(float(row[2]))

# Plot the data
plt.figure()
plt.plot(timestamps, cbf_0_values, label="h(x)")
plt.plot(timestamps, cbf_1_values, label=r"$\dot{h(x,u)} + \alpha h(x)$")
plt.xlabel("Time (seconds)")
plt.ylabel("CBF Values")
plt.legend()
plt.title("CBF Values vs Time")
plt.grid()
plt.show()
