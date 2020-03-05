/*********************************************************************
 *This is demo for ROS refering to xv_11_laser_driver.
 *********************************************************************/

#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>

namespace lh_laser_driver {
    class LHLaser {
        public:
	      uint16_t rpms; 
            LHLaser(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io);
            ~LHLaser() {};

            void poll(sensor_msgs::LaserScan::Ptr scan);

            void close() { shutting_down_ = true; };

        private:
            std::string port_; 
            uint32_t baud_rate_; 
            uint32_t firmware_; 

            bool shutting_down_; 
            boost::asio::serial_port serial_; 
	    uint16_t motor_speed_; 
    };
};
