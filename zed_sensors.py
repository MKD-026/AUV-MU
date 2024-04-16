import pyzed.sl as sl

# Create a Camera object
zed = sl.Camera()

init_params = sl.InitParameters()
init_params.camera_fps = 30

# Open the camera
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS :
    print(repr(err))
    zed.close()
    exit(1)

# Display camera information (model,S/N, fw version)
print("Camera Model: " + str(cam_model))
print("Serial Number: " + str(info.serial_number))
print("Camera Firmware: " + str(info.camera_configuration.firmware_version))
print("Sensors Firmware: " + str(info.sensors_configuration.firmware_version))

# Display sensors parameters (imu,barometer,magnetometer)
printSensorParameters(info.sensors_configuration.accelerometer_parameters) # accelerometer configuration
printSensorParameters(info.sensors_configuration.gyroscope_parameters) # gyroscope configuration
printSensorParameters(info.sensors_configuration.magnetometer_parameters) # magnetometer configuration
printSensorParameters(info.sensors_configuration.barometer_parameters) # barometer configuration

# Filtered orientation quaternion
quaternion = sensors_data.get_imu_data().get_pose().get_orientation().get()
print(" \t Orientation: [ Ox: {0}, Oy: {1}, Oz {2}, Ow: {3} ]".format(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

# linear acceleration
linear_acceleration = sensors_data.get_imu_data().get_linear_acceleration()
print(" \t Acceleration: [ {0} {1} {2} ] [m/sec^2]".format(linear_acceleration[0], linear_acceleration[1], linear_acceleration[2]))

# angular velocities
angular_velocity = sensors_data.get_imu_data().get_angular_velocity()
print(" \t Angular Velocities: [ {0} {1} {2} ] [deg/sec]".format(angular_velocity[0], angular_velocity[1], angular_velocity[2]))