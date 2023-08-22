#include "dynamixel_workbench_controllers/dynamixel_workbench_controllers.h"

double init_p1, init_p2, init_p3;
double ref_p1, ref_p2, ref_p3;
double deg2rad  = M_PI/180;

void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  init_p1 = msg->position.at(0);
  init_p2 = msg->position.at(1);
  init_p3 = msg->position.at(2); 
  
  //ROS_INFO("Initial position of M1: %.2f, M2: %.2f, M3: %.2f", init_p1, init_p2, init_p3);
}

// void refCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg){
//   ref_p1 = msg->points[0].positions.at(0);
//   ref_p2 = msg->points[0].positions.at(1);
//   ref_p3 = msg->points[0].positions.at(2);
// }

int main(int argc, char **argv)
{

  double i_p1 = init_p1;
  double i_p2 = init_p2;
  double i_p3 = init_p3;
  std::cout <<i_p1<<" "<<i_p2<<" "<<i_p3 << std::endl;
    //motor positions
  double lp_r = 100;
  int ilp_r = lp_r;
  int fall_p = ilp_r*10;
  int calib_p = ilp_r*8;

  double t_m;
  double l_t = 0.0025;
  double pos_t = 3;

  int size = ilp_r*12.5;
  int b;


  float pos1[size];
  float pos2[size];
  float pos3[size];

  int a = 200;

  std::cout<<lp_r<<" "<<ilp_r<<" "<<fall_p <<" "<<calib_p<<" "<<l_t <<" "<<pos_t <<" "<<size <<std::endl;

  std::cin>>b;
  
  //phase shift and amplitudes
  double phase = 120 * deg2rad;
  double amp1 = a * deg2rad;
  double amp2 = a * deg2rad;
  double amp3 = a * deg2rad;
  double pos = 120 * deg2rad;
  double w = 4*M_PI;

  //generate motor positions
  for(int i = 0; i!= size; ++i){
      pos1[i] = (amp1 * sin(w * i/size));// + init_p1;
      pos2[i] = (amp2 * sin(w * i/size + phase));//+init_p2;
      pos3[i] = (amp3 * sin(w * i/size + 2 * phase));//+init_p3;


  // std::cout <<pos1[i]<<" "<<pos2[i]<<" "<<pos3[i] << std::endl;
  }


    double g_pos1 = (amp1 * sin(w * 0/size));//0 * deg2rad;
    double g_pos2 = (amp2 * sin(w * 0/size + phase));//0 * deg2rad;
    double g_pos3 = (amp3 * sin(w * 0/size + 2*phase));//0 * deg2rad;


static const int numberOfMotors = 3;


ros::init(argc, argv, "pcNode");
ros::NodeHandle n;


  // initialize the subscriber
ros::Subscriber sub = n.subscribe("/dynamixel_workbench_controllers/joint_states", 1000, chatterCallback);//current position
ros::Rate con_rate(1);
con_rate.sleep();  
ros::spinOnce();
// ros::Subscriber sub2 = n.subscribe("/dynamixel_workbench_controllers/joint_trajectory", 1000, refCallback);
// ros::Rate con_rate2(100);
// con_rate2.sleep();  
// ros::spinOnce();
  //sub.shutdown();

  // initialize publisher
ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("/dynamixel_workbench_controllers/joint_trajectory", 200);
ros::Rate loop_rate(lp_r);


int count = 0;

trajectory_msgs::JointTrajectory motorCommand;
trajectory_msgs::JointTrajectoryPoint desiredConfiguration;

desiredConfiguration.positions.resize(numberOfMotors);  
desiredConfiguration.velocities.resize(numberOfMotors);
motorCommand.joint_names.resize(numberOfMotors);

// while()

while(ros::ok() && count<size+fall_p*2) {
  if(count<calib_p){
    desiredConfiguration.positions[0] = g_pos1+ i_p1;// + init_p1;
    //desiredConfiguration.velocities[0] = 1.0;
    motorCommand.joint_names[0] = "M1";

    desiredConfiguration.positions[1] = g_pos2+ i_p2;// + init_p2;
    //desiredConfiguration.velocities[1] = 1.0;
    motorCommand.joint_names[1] = "M2";

    desiredConfiguration.positions[2] = g_pos3+ i_p3;// + init_p3;
    //desiredConfiguration.velocities[2] = 1.0;
    motorCommand.joint_names[2] = "M3";
    t_m = pos_t;
  }

  if (count>=calib_p && count<fall_p){
    desiredConfiguration.positions[0] = g_pos1+ i_p1;// + init_p1;
    //desiredConfiguration.velocities[0] = 1.0;
    motorCommand.joint_names[0] = "M1";

    desiredConfiguration.positions[1] = g_pos2+ i_p2;// + init_p2;
    //desiredConfiguration.velocities[1] = 1.0;
    motorCommand.joint_names[1] = "M2";

    desiredConfiguration.positions[2] = g_pos3+ i_p3;// + init_p3;
    //desiredConfiguration.velocities[2] = 1.0;
    motorCommand.joint_names[2] = "M3";
    t_m = l_t;
  }

  if(count>=fall_p && count<(fall_p+size)){
    desiredConfiguration.positions[0] = pos1[count-fall_p]+ i_p1;// + init_p1;
    //desiredConfiguration.velocities[0] = 1.0;
    motorCommand.joint_names[0] = "M1";

    desiredConfiguration.positions[1] = pos2[count-fall_p]+ i_p2;// + init_p2;
    //desiredConfiguration.velocities[1] = 1.0;
    motorCommand.joint_names[1] = "M2";

    desiredConfiguration.positions[2] = pos3[count-fall_p]+ i_p3;// + init_p3;
    //desiredConfiguration.velocities[2] = 1.0;
    motorCommand.joint_names[2] = "M3";
    t_m = l_t;
  }

  if(count>=(fall_p+size)){
    desiredConfiguration.positions[0] = i_p1;// + init_p1;
    //desiredConfiguration.velocities[0] = 1.0;
    motorCommand.joint_names[0] = "M1";

    desiredConfiguration.positions[1] = i_p2;// + init_p2;
    //desiredConfiguration.velocities[1] = 1.0;
    motorCommand.joint_names[1] = "M2";

    desiredConfiguration.positions[2] = i_p3;// + init_p3;
    //desiredConfiguration.velocities[2] = 1.0;
    motorCommand.joint_names[2] = "M3";
    t_m = pos_t;
  }

    motorCommand.header.stamp = ros::Time::now();
    motorCommand.header.frame_id = "base_link";
    motorCommand.points.resize(1); // only one point so far
    motorCommand.points[0] = desiredConfiguration;
    motorCommand.points[0].time_from_start = ros::Duration(t_m); // 1 ns

    // cout << "sending command ..." << endl;
    std::cout << count  <<std::endl;
    // std::cout << count << std::endl;
    pub.publish(motorCommand);
    ros::spinOnce();
    loop_rate.sleep();

    count++;
  }





  return 0;

}

