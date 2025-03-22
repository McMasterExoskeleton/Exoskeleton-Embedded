import pandas as pd
import matplotlib.pyplot as plt
import os

# Get the absolute path for the script directory
script_dir = os.path.dirname(os.path.abspath(__file__))
csv_path = os.path.join(script_dir, "log_output.csv")

# Load the CSV log
df = pd.read_csv(csv_path)

# Skip the first 200ms of data to avoid the initial spike
df = df[df["time"] > 0.2].reset_index(drop=True)

# Hip Figure
fig_hip, axs_hip = plt.subplots(4, 1, figsize=(10, 8))
fig_hip.suptitle("Hip Joint Data", fontsize=16)

axs_hip[0].plot(df["time"], df["hip_angle"], color='blue')
axs_hip[0].set_title("Hip Angle (rad)")
axs_hip[0].set_ylabel("rad")

axs_hip[1].plot(df["time"], df["hip_velocity"], color='green')
axs_hip[1].set_title("Hip Velocity (rad/s)")
axs_hip[1].set_ylabel("rad/s")

axs_hip[2].plot(df["time"], df["hip_acceleration"], color='orange')
axs_hip[2].set_title("Hip Acceleration (rad/s²)")
axs_hip[2].set_ylabel("rad/s²")

axs_hip[3].plot(df["time"], df["hip_torque"], color='red')
axs_hip[3].set_title("Hip Torque (Nm)")
axs_hip[3].set_ylabel("Nm")
axs_hip[3].set_xlabel("Time (s)")

for ax in axs_hip:
    ax.grid(True)

fig_hip.tight_layout(rect=[0, 0.03, 1, 0.95])

# Knee Figure
fig_knee, axs_knee = plt.subplots(4, 1, figsize=(10, 8))
fig_knee.suptitle("Knee Joint Data", fontsize=16)

axs_knee[0].plot(df["time"], df["knee_angle"], color='blue')
axs_knee[0].set_title("Knee Angle (rad)")
axs_knee[0].set_ylabel("rad")

axs_knee[1].plot(df["time"], df["knee_velocity"], color='green')
axs_knee[1].set_title("Knee Velocity (rad/s)")
axs_knee[1].set_ylabel("rad/s")

axs_knee[2].plot(df["time"], df["knee_acceleration"], color='orange')
axs_knee[2].set_title("Knee Acceleration (rad/s²)")
axs_knee[2].set_ylabel("rad/s²")

axs_knee[3].plot(df["time"], df["knee_torque"], color='red')
axs_knee[3].set_title("Knee Torque (Nm)")
axs_knee[3].set_ylabel("Nm")
axs_knee[3].set_xlabel("Time (s)")

for ax in axs_knee:
    ax.grid(True)

fig_knee.tight_layout(rect=[0, 0.03, 1, 0.95])

# Torque Components
# (Mass Matrix * acceleration)
# Correalis
# Gravity compensation
fig_torque, axs_torque = plt.subplots(2, 1, figsize=(10, 6))
fig_torque.suptitle("Torque Component Breakdown", fontsize=16)

# Hip torque breakdown
axs_torque[0].plot(df["time"], df["hip_m_acc"], label="M * acc", color="purple")
axs_torque[0].plot(df["time"], df["hip_c_vel"], label="C * vel", color="orange")
axs_torque[0].plot(df["time"], df["hip_g"], label="Gravity", color="green")
axs_torque[0].plot(df["time"], df["hip_torque"], label="Total Torque", color="red", linestyle="--")
axs_torque[0].set_title("Hip Torque Components")
axs_torque[0].set_ylabel("Nm")
axs_torque[0].legend()
axs_torque[0].grid(True)

# Knee torque breakdown
axs_torque[1].plot(df["time"], df["knee_m_acc"], label="M * acc", color="purple")
axs_torque[1].plot(df["time"], df["knee_c_vel"], label="C * vel", color="orange")
axs_torque[1].plot(df["time"], df["knee_g"], label="Gravity", color="green")
axs_torque[1].plot(df["time"], df["knee_torque"], label="Total Torque", color="red", linestyle="--")
axs_torque[1].set_title("Knee Torque Components")
axs_torque[1].set_ylabel("Nm")
axs_torque[1].set_xlabel("Time (s)")
axs_torque[1].legend()
axs_torque[1].grid(True)

fig_torque.tight_layout(rect=[0, 0.03, 1, 0.95])

# Show all figures
plt.show()

