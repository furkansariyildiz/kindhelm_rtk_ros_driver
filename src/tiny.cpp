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



void Tiny::readTinyData(void)
{

}



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<Tiny>());
    return 0;
}