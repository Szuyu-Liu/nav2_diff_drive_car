#ifndef IMU_NODE_HPP
#define IMU_NODE_HPP


//#include <memory>
//#include <sys/ioctl.h>
//#include <fcntl.h>
//#include <unistd.h>
//#include <wiringPi.h>
//#include <wiringPiI2C.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <type_traits>
#include <functional>
#include <sys/ioctl.h>
#include <cstdio>
#include <linux/i2c-dev.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <eigen3/Eigen/Geometry>

extern "C"
{
#include "imu_package/BNO055.h"
#include <linux/i2c-dev.h>
//#include <linux/i2c.h>
//#include "smbus_functions.h"

}
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"

#define BNO055_ERROR ERROR1
using namespace std::chrono_literals;

	class ImuNode : public rclcpp::Node
	{

	public:
		ImuNode(int, char **);
		~ImuNode();
		void update_timer(int timeout);
		static BNO055_RETURN_FUNCTION_TYPE BNO_Init(struct bno055_t *);
		static BNO055_RETURN_FUNCTION_TYPE BNO055_I2C_bus_read(unsigned char, unsigned char, unsigned char *, unsigned char);
		static BNO055_RETURN_FUNCTION_TYPE BNO055_I2C_bus_write(unsigned char, unsigned char, unsigned char *, unsigned char);
		static void _delay(unsigned int);
		static void delayMicroseconds(unsigned int);
		void BnoCalibration(void);

		
	private:
		void timer_callback();

		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu;
		rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr imu_euler;

		struct bno055_t myBNO;
	 	struct bno055_euler myEulerData;
		struct bno055_quaternion myQuaternion;
		struct bno055_accel myAcceleration;
		struct bno055_gyro myGyro; 

		unsigned char accelCalibStatus; // Variable to hold the calibration status of the Accelerometer
		unsigned char magCalibStatus;	// Variable to hold the calibration status of the Magnetometer
		unsigned char gyroCalibStatus;	// Variable to hold the calibration status of the Gyroscope
		unsigned char sysCalibStatus;	// Variable to hold the calibration status of the System (BNO055's MCU)

		std_msgs::msg::Float32MultiArray euler_message;
		sensor_msgs::msg::Imu imu_data;		
	};

	static const char *filename = "/dev/i2c-3"; // PATH to I2C bus
	static int file = -1;



#endif // IMU_NODE_HPP
