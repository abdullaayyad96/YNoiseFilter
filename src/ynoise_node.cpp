#include <ros/ros.h>

#include "ynoise.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ynoise_filter");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ynoise::YNoiseFilter* filter = new ynoise::YNoiseFilter(nh, nh_private);

  ros::spin();
  return 0;
}
