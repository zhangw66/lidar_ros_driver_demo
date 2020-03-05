/*********************************************************************
 *This is demo for ROS refering to xv_11_laser_driver.
 rosrun LH_laser_driver LH_laser_publisher _frame_id:=LH_laser _port:=/dev/ttyUSB0 _baud_rate:=230400 _firmware_version:=2
echo LOCONH >/dev/ttyUSB0 //with intensity
echo LNCONH >/dev/ttyUSB0 //without intensity
echo LSTARH >/dev/ttyUSB0 //working begins
echo LSTOPH >/dev/ttyUSB0 //working stops
echo LRESTH >/dev/ttyUSB0 //working stops
echo LSRPM:xxxH >/dev/ttyUSB0 //set motor RPM
 *********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <lh_laser_driver/lh_laser.h>
#include <std_msgs/UInt16.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lh_laser_publisher");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  std::string port;
  int baud_rate;
  std::string frame_id;
 
  std_msgs::UInt16 rpms;

  priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
  priv_nh.param("baud_rate", baud_rate, 230400);
  priv_nh.param("frame_id", frame_id, std::string("lh_laser"));

  boost::asio::io_service io;
  try {
    lh_laser_driver::LHLaser laser(port, baud_rate, io);
    ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1500);
    //ros::Publisher motor_pub = n.advertise<std_msgs::UInt16>("rpms",1500);
		sleep(3);
    while (ros::ok()) {
      sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
      laser.poll(scan);
      scan->header.frame_id = frame_id;
      scan->header.stamp = ros::Time::now();      
      //rpms.data=laser.rpms;
      laser_pub.publish(scan);
      //motor_pub.publish(rpms);
    }
    laser.close();
    return 0;
  } catch (boost::system::system_error ex) {
    ROS_ERROR("LH_laser Error:%s", ex.what());
    return -1;
  }
}
