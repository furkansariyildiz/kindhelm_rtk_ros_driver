#include <kindhelm_rtk_ros_package/tiny.hpp>

using namespace Kindhelm;
using namespace std;



Tiny::Tiny():
Node("kindhelm_tiny_node")
{
    _read_timer = this->create_wall_timer(10ms, bind(&Tiny::readTinyData, this));

    _nav_sat_fix_publisher = this->create_publisher<sensor_msgs::msg::NavSatFix>("/fix", 1);

    _imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 1);

    declare_parameter("port_name", "/dev/ttyUSB0");
    declare_parameter("baudrate", B115200);

    _serial_port_name = this->get_parameter("port_name").as_string();
    _baudrate = this->get_parameter("baudrate").as_int();

    _gngga_pattern = regex("\\$GNGGA,([\\d.]+),([\\d.]+),([NS]),([\\d.]+),([EW]),(\\d+),(\\d+),([\\d.]+)?,([\\d.]+)?,([M])?,([\\d.]+)?,([M])?,([\\d.]+)?,([\\d.]+)?\\*");
    _pkhm_pattern =  regex("\\$PKHM,([\\d.]+),(\\d+),(\\d+),(\\d+),(\\d+.\\d+),(\\d+.\\d+),(\\d+.\\d+)\\*");
    _gnvtg_pattern = regex("\\$GNVTG,([\\d.]+),T,([\\d.]+),M,([\\d.]+),N,([\\d.]+),K,N\\*\\d+");

}



Tiny::~Tiny()
{

}


int Tiny::settingBaudrate(int baudrate)
{
    switch(baudrate)
    {
        case 9600:
            RCLCPP_INFO_STREAM(this->get_logger(), "Baudrate of serial port 9600");
            baudrate = B9600;
            break;

        case 19200:
            RCLCPP_INFO_STREAM(this->get_logger(), "Baudrate of serial port 19200");
            baudrate = B19200;
            break;

        case 38400:
            RCLCPP_INFO_STREAM(this->get_logger(), "Baudrate of serial port 38400");
            baudrate = B38400;
            break;

        case 57600:
            RCLCPP_INFO_STREAM(this->get_logger(), "Baudrate of serial port 57600");
            baudrate = B57600;
            break;

        case 115200:
            RCLCPP_INFO_STREAM(this->get_logger(), "Baudrate of serial port 115200");
            baudrate = B115200;
            break;

        default:
            RCLCPP_INFO_STREAM(this->get_logger(), "Default baudrate of serial port 9600");
            baudrate = B9600;
            break;
    }
    return baudrate;
}



void Tiny::openSerialPort(void)
{
    _serial_port = open(_serial_port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if(_serial_port < 0)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Can not open serial port...");
    }
}



void Tiny::prepareTermios(termios *tty, int serial_port, int baudrate)
{
    if(tcgetattr(serial_port, tty) != 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Can not open tiny!");
        rclcpp::shutdown();
    }

    cfsetospeed(tty, (speed_t)baudrate);
    cfsetispeed(tty, (speed_t)baudrate);

    tty->c_cflag     &=  ~PARENB;           
    tty->c_cflag     &=  ~CSTOPB;
    tty->c_cflag     &=  ~CSIZE;
    tty->c_cflag     |=  CS8;

    tty->c_cflag     &=  ~CRTSCTS;           
    tty->c_cc[VMIN]   =  1;                 
    tty->c_cc[VTIME]  =  5;                  
    tty->c_cflag     |=  CREAD | CLOCAL;     

    cfmakeraw(tty);
    tcflush(serial_port, TCIFLUSH);

    if(tcsetattr(serial_port, TCSANOW, tty) != 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Can not configure tiny!");
        rclcpp::shutdown();
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Tiny serial port configuration is completed.");
}



void Tiny::readTinyData(void)
{

}



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<Tiny>());
    return 0;
}