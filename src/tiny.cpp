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
    declare_parameter("baudrate", 115200);

    _serial_port_name = this->get_parameter("port_name").as_string();
    _baudrate = this->get_parameter("baudrate").as_int();

    _baudrate = this->settingBaudrate(_baudrate);
    
    this->openSerialPort();
    this->prepareTermios(&_tty, _serial_port, _baudrate);

    _gngga_pattern = regex("\\$GNGGA,([\\d.]+),([\\d.]+),([NS]),([\\d.]+),([EW]),(\\d+),(\\d+),([\\d.]+)?,([\\d.]+)?,([M])?,([\\d.]+)?,([M])?,([\\d.]+)?,([\\d.]+)?\\*");
    _pkhm_pattern =  regex("\\$PKHM,([\\d.]+),(\\d+),(\\d+),(\\d+),(\\d+.\\d+),(\\d+.\\d+),(\\d+.\\d+)\\*");
    _gnvtg_pattern = regex("\\$GNVTG,([\\d.]+),T,([\\d.]+),M,([\\d.]+),N,([\\d.]+),K,N\\*\\d+");
}



Tiny::~Tiny()
{
    close(_serial_port);
    RCLCPP_INFO_STREAM(this->get_logger(), "Tiny RTK Node is closed...");
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
        rclcpp::shutdown();
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
    _bytes = read(_serial_port, _buffer, sizeof(_buffer));

    if(_bytes != -1)
    {
        _readed_data = _readed_data + string(_buffer, _bytes);
        if(_readed_data.size() > 100)
        {
            string serial_data = regex_replace(_readed_data, regex(" "), "");
            makeSenseOfData(serial_data);
            _readed_data = "";
        }
    }
}



void Tiny::makeSenseOfData(string serial_data)
{
    _regex_iterator_start = serial_data.cbegin();
    _regex_iterator_end = serial_data.cend();

    while(regex_search(_regex_iterator_start, _regex_iterator_end, _gngga_matches, _gngga_pattern) || regex_search(_regex_iterator_start, _regex_iterator_end, _pkhm_matches, _pkhm_pattern) || regex_search(_regex_iterator_start, _regex_iterator_end, _gnvtg_matches, _gnvtg_pattern))
    {
        if(!(_gngga_matches.empty()))
        {
            string gngga_time = _gngga_matches[1].str();
            string latitude = _gngga_matches[2].str();
            string ns_indicator = _gngga_matches[3].str();
            string longitude = _gngga_matches[4].str();
            string ew_indicator = _gngga_matches[5].str();
            string fix_quality = _gngga_matches[6].str();
            string number_of_satellites = _gngga_matches[7].str();
            string hdop = _gngga_matches[8].str();
            string altitude = _gngga_matches[9].str();
            string altitude_unit = _gngga_matches[10].str();
            string geoid_separation = _gngga_matches[11].str();
            string geoid_separation_unit = _gngga_matches[12];
            string age_of_differential_correction = _gngga_matches[13].str();
            string differential_reference_station_id = _gngga_matches[14].str();

            _nav_sat_fix_message.latitude = degreesMinutesToDecimalDegrees(stod(latitude));
            _nav_sat_fix_message.longitude = degreesMinutesToDecimalDegrees(stod(longitude));
            _nav_sat_fix_message.altitude = degreesMinutesToDecimalDegrees(stod(altitude));

            RCLCPP_INFO_STREAM(this->get_logger(), "Latitude: " <<_nav_sat_fix_message.latitude);
            RCLCPP_INFO_STREAM(this->get_logger(), "Longitude: " << _nav_sat_fix_message.longitude);
            RCLCPP_INFO_STREAM(this->get_logger(), "Altitude: " << _nav_sat_fix_message.altitude);
            
            publishNavSatFix();
        }
        else if(!(_pkhm_matches.empty()))
        {
            string pkhm_time = _pkhm_matches[1].str();
            string pkhm_day = _pkhm_matches[2].str();
            string pkhm_month = _pkhm_matches[3].str();
            string pkhm_year = _pkhm_matches[4].str();

            _roll = stod(_pkhm_matches[5].str()) * PI / 180.0;
            _pitch = stod(_pkhm_matches[6].str()) * PI / 180.0;
            _yaw = stod(_pkhm_matches[7].str()) * PI / 180.0;

            _roll = atan2(sin(_roll), cos(_roll));
            _pitch = atan2(sin(_pitch), cos(_pitch));
            _yaw = atan2(sin(_yaw), cos(_yaw));

            RCLCPP_INFO_STREAM(this->get_logger(), "Roll: " << _roll);
            RCLCPP_INFO_STREAM(this->get_logger(), "Pitch: " << _pitch);
            RCLCPP_INFO_STREAM(this->get_logger(), "Yaw: " << _yaw);

            _imu_quaternion.setRPY(_roll, _pitch, _yaw);
            _imu_quaternion.normalize();

            publishImu();
        }
        else if(!(_gnvtg_matches.empty()))
        {
            double speed_as_knots = stod(_gnvtg_matches[1].str());
            double speed_as_kilometer_per_hour = stod(_gnvtg_matches[2].str());
            double total_speed_as_knots = stod(_gnvtg_matches[3].str());
            double total_speed_as_kilometer_per_hour = stod(_gnvtg_matches[4].str());
        }
    }
}



double Tiny::degreesMinutesToDecimalDegrees(double degrees_minutes)
{
    double degrees = floor(degrees_minutes / 100.0);
    double minutes = degrees_minutes - degrees * 100;
    double decimal_degrees = degrees + minutes / 60.0;
    return decimal_degrees;
}



void Tiny::publishNavSatFix(void)
{
    _nav_sat_fix_message.header.stamp = this->get_clock()->now();
    _nav_sat_fix_message.header.frame_id = "tiny";

    _nav_sat_fix_publisher->publish(_nav_sat_fix_message);
}



void Tiny::publishImu(void)
{
    _imu_message.header.stamp = this->get_clock()->now();
    _imu_message.header.frame_id = "tiny";

    _imu_message.orientation.x = _imu_quaternion[0];
    _imu_message.orientation.y = _imu_quaternion[1];
    _imu_message.orientation.z = _imu_quaternion[2];
    _imu_message.orientation.w = _imu_quaternion[3];

    _imu_publisher->publish(_imu_message);
}



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<Tiny>());
    return 0;
}