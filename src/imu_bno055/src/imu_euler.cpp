#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

#define BNO055_ADDRESS 0x28
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_EULER_H_LSB_ADDR 0x1A

#define OPERATION_MODE_NDOF 0x0C

struct EulerAngles {
    float heading;
    float roll;
    float pitch;
};

/*--------------------This is the IMU BNO055 class------------*/
class BNO055 {
public:
    BNO055(const char* i2c_device);
    ~BNO055();
    bool begin();
    EulerAngles readEulerAngles();

private:
    int i2c_fd;
    const char* device;
    bool writeByte(uint8_t reg, uint8_t value);
    bool readBytes(uint8_t reg, uint8_t* buffer, size_t length);
};

BNO055::BNO055(const char* i2c_device) : device(i2c_device), i2c_fd(-1) {}

BNO055::~BNO055() {
    if (i2c_fd >= 0) {
        close(i2c_fd);
    }
}

bool BNO055::begin() {
    if ((i2c_fd = open(device, O_RDWR)) < 0) {
        std::cerr << "Failed to open the i2c bus" << std::endl;
        return false;
    }

    if (ioctl(i2c_fd, I2C_SLAVE, BNO055_ADDRESS) < 0) {
        std::cerr << "Failed to acquire bus access and/or talk to slave." << std::endl;
        return false;
    }

    // Set operation mode to NDOF
    if (!writeByte(BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF)) {
        return false;
    }
    sleep(1); // Waiting for sensor to initialize
    return true;
}

bool BNO055::writeByte(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    if (write(i2c_fd, buffer, 2) != 2) {
        std::cerr << "Failed to write byte to the i2c bus." << std::endl;
        return false;
    }
    return true;
}

bool BNO055::readBytes(uint8_t reg, uint8_t* buffer, size_t length) {
    if (write(i2c_fd, &reg, 1) != 1) {
        std::cerr << "Failed to write the register address to the i2c bus." << std::endl;
        return false;
    }
    if (read(i2c_fd, buffer, length) != length) {
        std::cerr << "Failed to read bytes from the i2c bus." << std::endl;
        return false;
    }
    return true;
}

EulerAngles BNO055::readEulerAngles() {
    EulerAngles angles{0, 0, 0};
    uint8_t buffer[6];

    if (readBytes(BNO055_EULER_H_LSB_ADDR, buffer, 6)) {
        angles.heading = ((buffer[1] << 8) | buffer[0]) / 16.0;
        angles.roll = ((buffer[3] << 8) | buffer[2]) / 16.0;
        angles.pitch = ((buffer[5] << 8) | buffer[4]) / 16.0;
    }

    return angles;
}

/*-- Publishers --*/
rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr imu_euler;
rclcpp::TimerBase::SharedPtr timer_;
BNO055 sensor("/dev/i2c-1");

/*-------------This is the ROS Message class to send an IMU message----------*/
class IMU_Euler : public rclcpp::Node
{
public:
  
  IMU_Euler() : Node("IMU_Euler")
  {
    //BNO055 sensor("/dev/i2c-1");
    if (!sensor.begin()) 
        std::cerr << "Failed to initialize BNO055" << std::endl;
    else
	std::cout << "Initialisation BMO055 finished" << std::endl;
	
  /*-- Publisher --*/
  imu_euler = this->create_publisher<std_msgs::msg::Float32MultiArray>("/imu_euler",  
  	rclcpp::QoS(rclcpp::KeepLast(1)));
  timer_ = this->create_wall_timer(100ms, std::bind(&IMU_Euler::timer_callback, this)); 
  }

private:
  void timer_callback()
  {
  std_msgs::msg::Float32MultiArray euler_message;
  euler_message.data.clear();
  EulerAngles angles = sensor.readEulerAngles();

  euler_message.data.push_back(angles.heading); // in Degrees
  euler_message.data.push_back(angles.roll);
  euler_message.data.push_back(angles.pitch);
  imu_euler->publish(euler_message);
  }
  
};
/*----------This program starts the ROS loop-------------------*/
int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMU_Euler>());
    rclcpp::shutdown();
    
    return 0;
}
