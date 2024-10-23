#!/usr/bin/python
import rospy
from sensor_msgs.msg import Imu, MagneticField
import board
import adafruit_fxos8700
import adafruit_fxas21002c

# import Adafruit_FXOS8700
# import Adafruit_FXAS21002C


def imu_talker():
    rospy.init_node("adafruit_nxp_9dof_imu", anonymous=True)
    # accelerometer and gyroscope
    pub = rospy.Publisher(
        "imu_raw",
        Imu,
        queue_size=0,
    )
    # whether to publish magnetometer values (defaults to false to increase inertia data rate)
    should_publish_magnetometer = rospy.get_param("~publish_magnetometer", False)
    pub_magn = None
    if should_publish_magnetometer:
        # magnetometer
        pub_magn = rospy.Publisher(
            "imu_magnetometer",
            MagneticField,
            queue_size=0,
        )
    else:
        rospy.logwarn("NXP IMU will run in 6DOF mode!")
    # obtain I2C bus connection
    i2c = board.I2C()
    acc_magn_sensor = adafruit_fxos8700.FXOS8700(i2c)
    # acc_magn_sensor = Adafruit_FXOS8700.FXOS8700()
    rospy.logdebug("NXP-9DOF: acc magn ready")
    gyro_sensor = adafruit_fxas21002c.FXAS21002C(i2c)
    # gyro_sensor = Adafruit_FXAS21002C.FXAS21002C()
    rospy.logdebug("NXP-9DOF: gyro ready")
    # create IMU message buffer for data loop
    rosimu = Imu()
    rosimu.header.frame_id = "nxp_9dof_imu"
    # NOTE: attempt to stabilize the data rate to ~ 200 Hz
    # stable rate is the rate that can be held with other processes running
    # i.e. in empirical experiments, the raw data rate should never go
    # below the stable rate
    STABLE_RATE = rospy.get_param("stable_rate", 200)  # [Hz]
    # will not delay for longer than this,
    # might delay for shorter periods
    MAX_WAIT_TIME = 1.0 / STABLE_RATE  # [sec]
    last_update_time = rospy.Time.now()
    # presumed readout to ROS-publish time
    # TODO: when refactoring IMU node to class,
    # experimental perf timing can be conducted before looping
    READOUT_TIME = 1.0 / 295  # [sec]

    # data readout & publish loop
    while not rospy.is_shutdown():

        curr_update_time = rospy.Time.now()
        # compute how long to wait to keep the stable rate
        update_diff = (curr_update_time - last_update_time).to_sec()
        # if update took longer than stable rate, then update_diff < 0 [sec]
        # otherwise update_diff > 0 [sec]
        # NOTE: remove readout-time from delay for better rate convergence
        stable_rate_delay = max(0, MAX_WAIT_TIME - update_diff - READOUT_TIME)
        delay_duration = rospy.Duration.from_sec(stable_rate_delay)
        # now delay to obtain the stable rate
        rospy.sleep(delay_duration)
        # apply delay to current time and reset last update time
        curr_update_time += delay_duration
        last_update_time = curr_update_time
        # set message readout time
        rosimu.header.stamp = curr_update_time

        # read out the data and populate IMU message buffer
        gyro_x, gyro_y, gyro_z = gyro_sensor.gyroscope
        accel_x, accel_y, accel_z = acc_magn_sensor.accelerometer
        rosimu.angular_velocity.x = gyro_x
        rosimu.angular_velocity.y = gyro_y
        rosimu.angular_velocity.z = gyro_z
        rosimu.linear_acceleration.x = accel_x
        rosimu.linear_acceleration.y = accel_y
        rosimu.linear_acceleration.z = accel_z

        # publish the data
        pub.publish(rosimu)

        if pub_magn is not None:
            mag_x, mag_y, mag_z = acc_magn_sensor.magnetometer

            magn_msg = MagneticField()
            magn_msg.header.stamp = rospy.Time.now()
            magn_msg.header.frame_id = "nxp_9dof_imu"
            magn_msg.magnetic_field.x = mag_x
            magn_msg.magnetic_field.y = mag_y
            magn_msg.magnetic_field.z = mag_z
            pub_magn.publish(magn_msg)


if __name__ == "__main__":
    try:
        imu_talker()
    except rospy.ROSInterruptException:
        pass
