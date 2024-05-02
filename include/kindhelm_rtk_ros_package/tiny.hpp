#ifndef TINY_HPP
#define TINY_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <iostream>
#include <regex>
#include <libusb-1.0/libusb.h>

#include <sstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <features.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

using namespace std;

namespace Kindhelm
{
    class Tiny: public rclcpp::Node
    {
        private:

        public:
            Tiny();
            ~Tiny();

            rclcpp::TimerBase::SharedPtr _read_timer;

            rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr _nav_sat_fix_publisher;
            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_publisher;

            int settingBaudrate(int baudrate);

            void openSerialPort();

            void prepareTermios(termios *tty, int serial_port, int baudrate);

            void readTinyData(void);

            void makeSenseOfData(string serial_data);

            void publishNavSatFix(void);

            void publishImu(void);

            sensor_msgs::msg::NavSatFix _nav_sat_fix_message;
            sensor_msgs::msg::Imu _imu_message;

            tf2::Quaternion _imu_quaternion;

            struct termios _tty;

            int _serial_port;

            string _serial_port_name;

            int _baudrate;

            int _bytes = 0;
            
            int _spot = 0;

            char _buffer[256];

            regex _gngga_pattern;
            regex _pkhm_pattern;
            regex _gnvtg_pattern;
            regex _keyword_pattern;

            string::const_iterator _regex_iterator_start, _regex_iterator_end;

            string _readed_data = "";

            double _roll, _pitch, _yaw;
    };
}

#endif