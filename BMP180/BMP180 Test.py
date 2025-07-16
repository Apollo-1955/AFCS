from machine import Pin, I2C
from time import sleep
import bmp180

# Setup I2C (adjust pins if needed)
i2c = I2C(1, scl=Pin(3), sda=Pin(2), freq=100000)  # Slower for more reliability

# Initialize placeholder for sensor
sensor = None

# Try to initialize BMP180 until successful
while sensor is None:
    try:
        print("Attempting to connect to BMP180...")
        sensor = bmp180.BMP180(i2c)
        sensor.oversample_sett = 2
        sensor.sea_level_pressure = 101325  # Pa
        print("BMP180 connected successfully!")
    except Exception as e:
        print("Connection failed:", e)
        sleep(2)

# Loop to read data, reconnect on error
while True:
    try:
        sensor.measure()
        temp = sensor.temperature
        pressure = sensor.pressure
        altitude = sensor.altitude

        print("Temperature: {:.2f} °C".format(temp))
        print("Pressure: {:.2f} Pa".format(pressure))
        print("Altitude: {:.2f} m".format(altitude))
        print("---------------------------")

    except Exception as e:
        print("Read failed:", e)
        print("Reinitializing sensor...")
        sensor = None
        # Retry connection
        while sensor is None:
            try:
                sensor = bmp180.BMP180(i2c)
                sensor.oversample_sett = 2
                sensor.sea_level_pressure = 101325
                print("Reconnected to BMP180!")
            except Exception as e:
                print("Reconnection failed:", e)
                sleep(2)
    sleep(0.5)