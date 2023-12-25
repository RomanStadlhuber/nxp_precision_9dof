#!/usr/bin/python
import rospy
from sensor_msgs.msg import Imu, MagneticField
import board
import adafruit_fxos8700
import adafruit_fxas21002c

# import Adafruit_FXOS8700
# import Adafruit_FXAS21002C


def imu_talker():
    # accelerometer and gyroscope
    pub = rospy.Publisher(
        "/imu_raw",
        Imu,
        queue_size=10,
    )
    # magnetometer
    pub_magn = rospy.Publisher(
        "/imu_magnetometer",
        MagneticField,
        queue_size=10,
    )
    rospy.init_node("adafruit_nxp_9dof_imu", anonymous=True)
    rate = rospy.Rate(100)  # [Hz]
    # obtain I2C bus connection
    i2c = board.I2C()
    acc_magn_sensor = adafruit_fxos8700.FXOS8700(i2c)
    # acc_magn_sensor = Adafruit_FXOS8700.FXOS8700()
    rospy.loginfo("acc magn ready")
    gyro_sensor = adafruit_fxas21002c.FXAS21002C(i2c)
    # gyro_sensor = Adafruit_FXAS21002C.FXAS21002C()
    rospy.loginfo("gyro ready")
    while not rospy.is_shutdown():
        rosimu = Imu()

        gyro_x, gyro_y, gyro_z = gyro_sensor.gyroscope
        accel_x, accel_y, accel_z = acc_magn_sensor.accelerometer
        mag_x, mag_y, mag_z = acc_magn_sensor.magnetometer

        rosimu.header.stamp = rospy.Time.now()
        rosimu.header.frame_id = "imu_raw"

        rosimu.angular_velocity.x = gyro_x
        rosimu.angular_velocity.y = gyro_y
        rosimu.angular_velocity.z = gyro_z

        rosimu.linear_acceleration.x = accel_x
        rosimu.linear_acceleration.y = accel_y
        rosimu.linear_acceleration.z = accel_z

        magn_msg = MagneticField()
        magn_msg.header.stamp = rospy.Time.now()
        magn_msg.header.frame_id = "imu_raw"
        magn_msg.magnetic_field.x = mag_x
        magn_msg.magnetic_field.y = mag_y
        magn_msg.magnetic_field.z = mag_z

        pub.publish(rosimu)
        pub_magn.publish(magn_msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        imu_talker()
    except rospy.ROSInterruptException:
        pass
