import numpy as np
import datetime
import time

gyroTimestamps = []
gyroValues = []


def turnIntegral(gyroTimestamps, gyroValues):
    # # Check if there are enough data points to perform integration
    # if len(gyroValues) < 2:
    #     return None

    # Calculate time differences in seconds
    time_diff = np.diff([t.astype(int) for t in gyroTimestamps])
    time_diff_seconds = time_diff / 1e6  # Convert nanoseconds to seconds
    print(time_diff_seconds)

    if len(gyroValues) == 0 or len(time_diff_seconds) == 0:
        return 0
    return time_diff_seconds[-1] * gyroValues[-1]

    # Integrate the data values using the trapezoidal rule
    integral = np.trapz(gyroValues[:-1], x=time_diff_seconds)

    return integral


if __name__ == "__main__":
    for i in range(500, 600, 1):
        # Get the current datetime
        current_time = datetime.datetime.now()

        # Convert the current datetime to np.datetime64
        current_time_np = np.datetime64(current_time)

        gyroTimestamps.append(current_time_np)
        gyroValues.append(i)
        time.sleep(0.1)
