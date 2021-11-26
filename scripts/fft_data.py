import rospy
import numpy as np
import matplotlib.pyplot as plt
from rospy.rostime import Duration
from sensor_msgs.msg import Imu
import pathlib

data = {
    "gyro": {
        "x": [],
        "y": [],
        "z": []
    },
    "accel": {
        "x": [],
        "y": [],
        "z": []
    },
}
ts = 0.

def save_fft_plot(data: list, dt: Duration, filename: str):
    Y    = np.fft.fft(np.array(data))
    freq = np.fft.fftfreq(len(data), dt.to_sec())
    plt.figure()
    plt.plot( freq, np.abs(Y) )
    plt.savefig("%s.png"%filename)
    print("saved to %s"%pathlib.Path().resolve())
    data.clear()

def data_cb(args):
    global ts
    global data

    imu_data = args
    data["gyro"]["x"].append(imu_data.angular_velocity.x)
    data["gyro"]["y"].append(imu_data.angular_velocity.y)
    data["gyro"]["z"].append(imu_data.angular_velocity.z)
    data["accel"]["x"].append(imu_data.linear_acceleration.x)
    data["accel"]["y"].append(imu_data.linear_acceleration.y)
    data["accel"]["z"].append(imu_data.linear_acceleration.z)

    if len(data["gyro"]["x"]) == 100:
        dt = args.header.stamp - ts
        save_fft_plot(data["gyro"]["x"],dt,"gyro_x")
        save_fft_plot(data["gyro"]["y"],dt,"gyro_y")
        save_fft_plot(data["gyro"]["z"],dt,"gyro_z")
        save_fft_plot(data["accel"]["x"],dt,"accel_x")
        save_fft_plot(data["accel"]["y"],dt,"accel_y")
        save_fft_plot(data["accel"]["z"],dt,"accel_z")

    ts = args.header.stamp

def main():
    rospy.init_node("fft_analysis")
    rospy.Subscriber("/mavros1/imu/data", Imu, data_cb)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass