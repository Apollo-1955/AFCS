import csv
import matplotlib.pyplot as plt

def plot_rocket_data(csv_filename):
    times = []
    altitude = []
    roll = []
    velocity = []
    servo_angle = []

    # Open and read CSV
    with open(csv_filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Parse time as float seconds.milliseconds
            try:
                t = float(row['Time (s)'])
            except:
                continue  # skip if can't parse

            times.append(t)

            # Convert values or set None if not valid
            def safe_float(val):
                try:
                    return float(val)
                except:
                    return None

            altitude.append(safe_float(row['Altitude (m)']))
            roll.append(safe_float(row['Roll (deg)']))
            velocity.append(safe_float(row['Velocity (m/s)']))
            servo_angle.append(safe_float(row['Servo Angle (deg)']))

    # Plot
    plt.figure(figsize=(12, 8))

    plt.plot(times, altitude, label='Altitude (m)')
    plt.plot(times, roll, label='Roll (deg)')
    plt.plot(times, velocity, label='Velocity (m/s)')
    plt.plot(times, servo_angle, label='Servo Angle (deg)')

    plt.xlabel('Time (s)')
    plt.title('Rocket Flight Data')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python plot_rocket_data.py <csv_filename>")
    else:
        plot_rocket_data(sys.argv[1])
