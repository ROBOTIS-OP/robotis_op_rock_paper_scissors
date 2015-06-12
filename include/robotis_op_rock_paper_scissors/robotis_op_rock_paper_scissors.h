#ifndef HECTOR_QRCODE_DETECTION_H
#define HECTOR_QRCODE_DETECTION_H

#include <ros/ros.h>
#include <hector_worldmodel_msgs/ImagePercept.h>

namespace zbar {
  class ImageScanner;
}

namespace robotis_op {

class robotis_op_rock_paper_scissors {
public:
  robotis_op_rock_paper_scissors(ros::NodeHandle nh, ros::NodeHandle priv_nh);
  ~robotis_op_rock_paper_scissors();

protected:
  void perceptCallback(const hector_worldmodel_msgs::ImagePercept& percept);

private:
  ros::NodeHandle nh_;
  ros::Subscriber percept_subscriber_;


  ros::Publisher j_shoulder_l_publisher_;
  ros::Publisher j_shoulder_r_publisher_;
  ros::Publisher j_high_arm_l_publisher_;
  ros::Publisher j_high_arm_r_publisher_;
  ros::Publisher j_low_arm_l_publisher_;
  ros::Publisher j_low_arm_r_publisher_;
  ros::Publisher j_tilt_publisher_;

  enum State {INIT, WATING_FOR_PERCEPT, PROCESSING_PERCEPT};
  enum Choice {NO_CHOICE, ROCK, PAPER, SCISSORS};
  enum Result {PLAYER_WINS, DRAW, ROBOT_WINS};
  static const std::string choices[4];

  void initialize();
  void waitForPercept();
  void processPercept(Choice player_choice);
  void moveToRock();
  void moveToPaper();
  void moveToScissor();
  void moveToOrigin();
  int computeResult(Choice player_choice, Choice robot_choice);


  int score_player;
  int score_robot;



  State state;
};

}
#endif
