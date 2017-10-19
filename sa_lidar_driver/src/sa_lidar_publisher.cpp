#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <sa_lidar_publisher.h>
#include <std_msgs/UInt16.h>


namespace dy_lidar
{
DYlaser::DYlaser(boost::asio::io_service& io)
: serial_(io)
  //,shutting_down_(false)
{
  nh_.param("port", port_, std::string("/dev/ttyUSB0"));
  nh_.param("baud_rate", baud_rate_, 115200);
  nh_.param("frame_id", frame_id_, std::string("laser"));


  serial_.open(port_);
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));


  scan_.header.frame_id = frame_id_;
  scan_.angle_min = 0.0;
  scan_.angle_max = M_PI;         // 라디안 단위
  scan_.angle_increment = (M_PI/200.0);    // 라디안 분해능
  scan_.range_min = 0.12;
  scan_.range_max = 20;			// 실제 최고 거리보다 작게해야 될듯.  20m 넘어가면 scan 값 그냥 뛰어넘음 rviz에 표시안됨
  scan_.ranges.resize(200);


  laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1000);

  ////////////////////////////////////////////////////////////////////////////////
  boost::array<uint8_t, 4> a = {0x02, 0x01, 0x00, 0x03};
  boost::asio::write(serial_, boost::asio::buffer(a));        // 라이다 시작 커맨드 보내기
}

void DYlaser::poll()
{

  bool got_scan = false;
  uint8_t start_count = 0;
  uint8_t good_sets = 0;    // 제대로 set이 들어왔을 때

  boost::array<uint8_t, 403> raw_bytes;     // 403 sets로 보냄 02 00 ~ 400개 ~ 03



  //while (!shutting_down_ && !got_scan)
  while (!got_scan)
  {
    boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[start_count],1));
    // raw_bytes에 시리얼 데이터 넘어온거 저장
    // 403개가 다 들어올때까지 기다림
    // 잘못들어온것도 저장하고 있다가 403개가 채워지면 raw_bytes에 저장됨...
    // 그래서 처음에 FA if 문 한듯.....
    // 시리얼값이 넘어올 때까지 정지해있음


    if(start_count == 0)
    {
      if(raw_bytes[start_count] == 0x02)
      {
        start_count = 1;
      }
    }
    else if(start_count == 1)
    {
      if(raw_bytes[start_count] == 0x00)   // 02 01 ~~400개~ 03
      {
            boost::asio::read(serial_,boost::asio::buffer(&raw_bytes[2], 401));

        if(raw_bytes[402] == 0x03)
        {
          start_count = 0;
          got_scan = true;

            int degree_count_num = 0;

            for(int j = 2; j < 401; j = j + 2)
            {
              uint8_t byte0 = raw_bytes[j];
              uint8_t byte1 = raw_bytes[j+1];

              uint16_t range     = (byte0 << 8) + byte1;

              if(range > 20000)
                {
                  range = 0;
                }

              //scan_.ranges[degree_count_num] = range / 1000.0;
              scan_.ranges[degree_count_num] = (range / 1000.0) - 7.2;  //offset 7.4m

              degree_count_num++;
            }

            scan_.time_increment = 0.1 / 200;   // 1sec / 10scan/ 200beam
            scan_.scan_time = 0.1;        // 10HZ scan time

            scan_.header.stamp = ros::Time::now();
            laser_pub_.publish(scan_);
            ros::spinOnce();

      }
      else
      {
        start_count = 0;
      }
    }
      else
      {
        start_count = 0;
      }

    }


  }


}

/*
void LFCDLaser::close()
{
  shutting_down_ = true;
  serial_.open(port_);
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  boost::asio::write(serial_, boost::asio::buffer("e", 1));
};
*/

};

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "sa_lidar_publisher");

  boost::asio::io_service io;
  dy_lidar::DYlaser laser(io);



  while (ros::ok())
  {
    laser.poll();
  }

  //laser.close();        // 라이다 멈춤

  return 0;
}
