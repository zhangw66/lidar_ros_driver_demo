/*********************************************************************
  *This is demo for ROS refering to xv_11_laser_driver.
 *********************************************************************/

#include <lh_laser_driver/lh_laser.h>
#include "boost/multi_array.hpp"  
#include<time.h>
namespace lh_laser_driver {
  LHLaser::LHLaser(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io):
		port_(port),
  	baud_rate_(baud_rate),
		shutting_down_(false),
		serial_(io, port_)
	{
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    serial_.set_option(  boost::asio::serial_port::flow_control( boost::asio::serial_port::flow_control::none ) ); 
    serial_.set_option(  boost::asio::serial_port::parity( boost::asio::serial_port::parity::none ) );
    serial_.set_option( boost::asio::serial_port::stop_bits(  boost::asio::serial_port::stop_bits::one ) );       
    serial_.set_option(  boost::asio::serial_port::character_size( 8 ) );                       
	}

  void LHLaser::poll(sensor_msgs::LaserScan::Ptr scan) {
		const int32_t MAX_COUNT_PER_MSG = 200;
    int32_t got_msg = 0;
    uint8_t buf[ 6 + 200 * 2 + 2] = {0};
    uint16_t data_count = 0;
    uint16_t angle_begin = 0;
    uint16_t ranges[MAX_COUNT_PER_MSG];
    uint16_t crc_recv = 0;
    uint16_t crc_calc = 0;

    while ((!shutting_down_) && (!got_msg)) {
      boost::asio::read(serial_, boost::asio::buffer(&buf[0], 1));
      if (buf[0] != 0xCE) {
        continue;
      }

      boost::asio::read(serial_, boost::asio::buffer(&buf[1], 1));
      if (buf[1] != 0xFA) {
        continue;
      }

      crc_calc = 0;
      boost::asio::read(serial_, boost::asio::buffer(&buf[2], 2));
      data_count = buf[2] | (buf[3] << 8);
      crc_calc += data_count;
     // printf("data count: %d\n", data_count);
      if (data_count > MAX_COUNT_PER_MSG) {
        continue;
      }

      boost::asio::read(serial_, boost::asio::buffer(&buf[4], 2));
      angle_begin = buf[4] | (buf[5] << 8);
      crc_calc += angle_begin;
	   printf("angle begin: %d\n", angle_begin);

      boost::asio::read(serial_, boost::asio::buffer(&buf[6], data_count * 2 + 2));
      for (int i = 0; i < data_count; i++) {
        ranges[i] = buf[6 + i * 2] | (buf[6 + i * 2 + 1] << 8);
        crc_calc += ranges[i];
      }

      crc_recv = buf[6 + data_count * 2] | (buf[6 + data_count * 2 + 1] << 8);
      if (crc_calc != crc_recv) {
        printf("crc error, calc: %u != recv: %u\n", crc_calc, crc_recv);
      }
      
      scan->angle_min = angle_begin / 10.0 * M_PI / 180;
      scan->angle_max = (angle_begin / 10.0 +  36) * M_PI / 180;
      scan->angle_increment = (scan->angle_max - scan->angle_min) / data_count;
      scan->time_increment = 1.0 / 10 / data_count; 
      scan->range_min = 0;
      scan->range_max = 30.0;
      scan->ranges.resize(data_count);
      scan->intensities.resize(data_count);
      
      for (int i = 0; i < data_count; i++) {
        ranges[i] = buf[6 + i * 2] | (buf[6 + i * 2 + 1] << 8);
        crc_calc += ranges[i];
        scan->ranges[i] = (ranges[i] & 0x1FFF) / 100.0;
        scan->intensities[i] = (ranges[i] & 0xE000) >> 13;
      }
      got_msg = 1;
    }
  }
};
