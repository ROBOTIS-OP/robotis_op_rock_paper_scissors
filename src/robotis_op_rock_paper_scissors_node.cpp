#include <robotis_op_rock_paper_scissors/robotis_op_rock_paper_scissors.h>

using namespace robotis_op;

int main(int argc, char **argv)
{

  ros::init(argc, argv, ROS_PACKAGE_NAME);
  robotis_op_rock_paper_scissors rock_paper_scissors(ros::NodeHandle(), ros::NodeHandle("~"));
  ros::spin();
  return 0;
}

