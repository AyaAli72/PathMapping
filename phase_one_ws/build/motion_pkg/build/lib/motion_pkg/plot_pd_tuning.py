import matplotlib.pyplot as plt
import pandas as pd

# Load data from CSV
df = pd.read_csv("/home/kiro/Music/phase_one_ws/pd_tuning_log.csv")

# Check column names (in case of errors)
print(df.head())

# Plot distance error over time
plt.figure(figsize=(10, 5))
plt.plot(df["time"], df["distance_error"], label="Distance Error", color="b")
plt.plot(df["time"], df["angle_error"], label="Angle Error", color="r")
plt.xlabel("Time (s)")
plt.ylabel("Error")
plt.title("PD Controller Error Over Time")
plt.legend()
plt.grid()
plt.show()
