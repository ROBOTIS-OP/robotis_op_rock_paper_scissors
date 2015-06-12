
#include <robotis_op_rock_paper_scissors/robotis_op_rock_paper_scissors.h>
#include <iostream>

#include <stdlib.h>     /* srand, rand */
#include <std_msgs/Float64.h>


namespace robotis_op {
const std::string robotis_op_rock_paper_scissors::choices[4] =
{
    "NO CHOICE", "ROCK", "PAPER", "SCISSORS"
};

robotis_op_rock_paper_scissors::robotis_op_rock_paper_scissors(ros::NodeHandle nh, ros::NodeHandle priv_nh)
    : nh_(nh)
{    
    percept_subscriber_ = nh_.subscribe("image_percept", 1, &robotis_op_rock_paper_scissors::perceptCallback, this);
    j_shoulder_l_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_shoulder_l_position_controller/command",1);
    j_shoulder_r_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_shoulder_r_position_controller/command",1);
    j_high_arm_l_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_high_arm_l_position_controller/command",1);
    j_high_arm_r_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_high_arm_r_position_controller/command",1);
    j_low_arm_l_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_low_arm_l_position_controller/command",1);
    j_low_arm_r_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_low_arm_r_position_controller/command",1);
    j_tilt_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_tilt_position_controller/command",1);
    initialize();
}

robotis_op_rock_paper_scissors::~robotis_op_rock_paper_scissors()
{
}


void robotis_op_rock_paper_scissors::initialize()
{

    ros::Duration shortdur(0.3);
    state = INIT;
    score_player = 0;
    score_robot = 0;
    shortdur.sleep();
    moveToOrigin();
    shortdur.sleep();
    ros::spinOnce();
    shortdur.sleep();
    ROS_INFO("Let's play ROCK PAPER SCISSORS");
    ROS_INFO("The first player with 3 points wins");
    ROS_INFO("Press Enter to start");
    std::cin.get();

    ROS_INFO("ROCK...");
    shortdur.sleep();
    ROS_INFO("PAPER...");
    shortdur.sleep();
    ROS_INFO("SCISSORS...");
    shortdur.sleep();
    state = WATING_FOR_PERCEPT;
}


void robotis_op_rock_paper_scissors::processPercept(Choice player_choice)
{

    srand(time(NULL));
    Choice robot_choice = Choice((rand()%3)+1); //gives 1,2 or 3
    ROS_INFO("Robot %s, Player %s",choices[robot_choice].c_str(),choices[player_choice].c_str());
    int res = computeResult(player_choice,robot_choice);
    if(res == PLAYER_WINS)
    {
        ROS_INFO("Player wins\n");
    }
    else if(res == ROBOT_WINS)
    {
        ROS_INFO("Robot wins\n");
    }
    else //draw
    {
        ROS_INFO("Draw\n");
    }


    if(robot_choice == ROCK)
    {
        moveToRock();
    }
    else if(robot_choice == SCISSORS)
    {
        moveToScissor();
    }
    else if(robot_choice == PAPER)
    {
        moveToPaper();
    }

    ros::spinOnce();
    ros::Duration dur(1.2);
    dur.sleep();

    moveToOrigin();

    ros::spinOnce();
    dur.sleep();

    dur.sleep();
    ros::spinOnce();

    ros::Duration shortdur(0.4);
    ROS_INFO("ROCK...");
    shortdur.sleep();
    ROS_INFO("PAPER...");
    shortdur.sleep();
    ROS_INFO("SCISSORS...");
    shortdur.sleep();
    state = WATING_FOR_PERCEPT;
}


int robotis_op_rock_paper_scissors::computeResult(Choice player_choice, Choice robot_choice)
{
    if(player_choice == robot_choice)
        return DRAW;
    else if (player_choice == ROCK && robot_choice == SCISSORS)
        return PLAYER_WINS;
    else if (player_choice == ROCK && robot_choice == PAPER)
        return ROBOT_WINS;
    else if (player_choice == SCISSORS && robot_choice == PAPER)
        return PLAYER_WINS;
    else if (player_choice == SCISSORS && robot_choice == ROCK)
        return ROBOT_WINS;
    else if (player_choice == PAPER && robot_choice == ROCK)
        return PLAYER_WINS;
    else if (player_choice == PAPER && robot_choice == SCISSORS)
        return ROBOT_WINS;
    else
    {
        ROS_ERROR("computeResult: Unknown constellation %i %i",player_choice,robot_choice);
    }
}

void robotis_op_rock_paper_scissors::perceptCallback(const hector_worldmodel_msgs::ImagePercept& percept)
{
    Choice player_choice = Choice(0);
    if(state == WATING_FOR_PERCEPT)
    {
        state=PROCESSING_PERCEPT;
        player_choice = Choice(atoi(percept.info.name.c_str()));
        processPercept(player_choice);
    }
}


void robotis_op_rock_paper_scissors::moveToRock()
{
    std_msgs::Float64 j_shoulder_l_msg;
    std_msgs::Float64 j_shoulder_r_msg;
    std_msgs::Float64 j_high_arm_l_msg;
    std_msgs::Float64 j_high_arm_r_msg;
    std_msgs::Float64 j_low_arm_l_msg;
    std_msgs::Float64 j_low_arm_r_msg;
    j_shoulder_l_msg.data = 0.0;
    j_shoulder_l_msg.data = 0.0;
    j_high_arm_l_msg.data = 0.6854;
    j_high_arm_r_msg.data = -0.6854;
    j_low_arm_l_msg.data = -0.9854;
    j_low_arm_r_msg.data = 0.9854;
    j_shoulder_l_publisher_.publish(j_shoulder_l_msg);
    j_shoulder_r_publisher_.publish(j_shoulder_r_msg);
    j_high_arm_l_publisher_.publish(j_high_arm_l_msg);
    j_high_arm_r_publisher_.publish(j_high_arm_r_msg);
    j_low_arm_l_publisher_.publish(j_low_arm_l_msg);
    j_low_arm_r_publisher_.publish(j_low_arm_r_msg);
}
void robotis_op_rock_paper_scissors::moveToPaper()
{
    std_msgs::Float64 j_shoulder_l_msg;
    std_msgs::Float64 j_shoulder_r_msg;
    std_msgs::Float64 j_high_arm_l_msg;
    std_msgs::Float64 j_high_arm_r_msg;
    std_msgs::Float64 j_low_arm_l_msg;
    std_msgs::Float64 j_low_arm_r_msg;
    j_shoulder_l_msg.data = 0.0;
    j_shoulder_r_msg.data = 0.0;
    j_high_arm_l_msg.data = -0.7854;
    j_high_arm_r_msg.data = 0.7854;
    j_low_arm_l_msg.data = -0.7854;
    j_low_arm_r_msg.data = 0.7854;
    j_shoulder_l_publisher_.publish(j_shoulder_l_msg);
    j_shoulder_r_publisher_.publish(j_shoulder_r_msg);
    j_high_arm_l_publisher_.publish(j_high_arm_l_msg);
    j_high_arm_r_publisher_.publish(j_high_arm_r_msg);
    j_low_arm_l_publisher_.publish(j_low_arm_l_msg);
    j_low_arm_r_publisher_.publish(j_low_arm_r_msg);
}
void robotis_op_rock_paper_scissors::moveToScissor()
{
    std_msgs::Float64 j_shoulder_l_msg;
    std_msgs::Float64 j_shoulder_r_msg;
    std_msgs::Float64 j_high_arm_l_msg;
    std_msgs::Float64 j_high_arm_r_msg;
    std_msgs::Float64 j_low_arm_l_msg;
    std_msgs::Float64 j_low_arm_r_msg;
    j_shoulder_l_msg.data = 0.0;
    j_shoulder_r_msg.data = 0.0;
    j_high_arm_l_msg.data = 2.5*-0.785398163;
    j_high_arm_r_msg.data = 2.5*0.785398163;
    j_low_arm_l_msg.data = 2*0.785398163;
    j_low_arm_r_msg.data = 2*-0.785398163;
    j_shoulder_l_publisher_.publish(j_shoulder_l_msg);
    j_shoulder_r_publisher_.publish(j_shoulder_r_msg);
    j_high_arm_l_publisher_.publish(j_high_arm_l_msg);
    j_high_arm_r_publisher_.publish(j_high_arm_r_msg);
    j_low_arm_l_publisher_.publish(j_low_arm_l_msg);
    j_low_arm_r_publisher_.publish(j_low_arm_r_msg);
}


void robotis_op_rock_paper_scissors::moveToOrigin()
{
    std_msgs::Float64 j_shoulder_l_msg;
    std_msgs::Float64 j_shoulder_r_msg;
    std_msgs::Float64 j_high_arm_l_msg;
    std_msgs::Float64 j_high_arm_r_msg;
    std_msgs::Float64 j_low_arm_l_msg;
    std_msgs::Float64 j_low_arm_r_msg;
    std_msgs::Float64 j_tilt_msg;
    j_shoulder_l_msg.data = 0.78539;
    j_shoulder_r_msg.data = -0.78539;
    j_high_arm_l_msg.data = 0.307177;
    j_high_arm_r_msg.data = -0.307177;
    j_low_arm_l_msg.data = -0.51605;
    j_low_arm_r_msg.data = 0.51605;
    j_tilt_msg.data = -0.3;
    j_shoulder_l_publisher_.publish(j_shoulder_l_msg);
    j_shoulder_r_publisher_.publish(j_shoulder_r_msg);
    j_high_arm_l_publisher_.publish(j_high_arm_l_msg);
    j_high_arm_r_publisher_.publish(j_high_arm_r_msg);
    j_low_arm_l_publisher_.publish(j_low_arm_l_msg);
    j_low_arm_r_publisher_.publish(j_low_arm_r_msg);
    j_tilt_publisher_.publish(j_tilt_msg);
}
}
