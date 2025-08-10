import pandas as pd
import matplotlib.pyplot as plt

# Define the file path
file_path = 'x.txt'  # Update this with your file path

# Initialize lists to hold the parsed data
times = []
set_points = []
process_values = []

# Read the log file and parse each line
with open(file_path, 'r') as file:
    for line in file:
        # Split the line into tokens
        tokens = line.split()
        if len(tokens) >= 3:
            # Extract time, set point, and process value
            time_str = tokens[0]
            sp_str = tokens[1].split(':')[1]
            pv_str = tokens[2].split(':')[1]
            
            times.append(pd.to_datetime(time_str, format='%H:%M:%S.%f'))
            set_points.append(float(sp_str))
            process_values.append(float(pv_str))

# Create a DataFrame from the parsed lists
data = pd.DataFrame({
    'time': times,
    'sp': set_points,
    'pv': process_values
})

# Display the DataFrame (optional)
print(data)

# Plot the data
plt.figure(figsize=(12, 6))
plt.plot(data['time'], data['sp'], label='Set Point (sp)', marker='o')
plt.plot(data['time'], data['pv'], label='Process Value (pv)', marker='x')
plt.title('Set Point vs. Process Value over Time')
plt.xlabel('Time')
plt.ylabel('Value')
plt.legend()
plt.grid()
plt.xticks(rotation=45)
plt.tight_layout()
plt.show()
