#ifndef SA_LIDAR_PUBLISHER_H_
#define SA_LIDAR_PUBLISHER_H_

#include <std_msgs/UInt16.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>

namespace dy_lidar
{
class DYlaser
 {
 public:

	DYlaser(boost::asio::io_service& io);


  ~DYlaser() {};


  void poll();



  //void close();

 private:
  ros::NodeHandle nh_;

  ros::Publisher laser_pub_;

  std::string frame_id_;
  sensor_msgs::LaserScan scan_;

  std::string port_;
  int baud_rate_;
  
  boost::asio::serial_port serial_;

  };
 };

#endif
