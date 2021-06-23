#include "ros/ros.h"
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "ros/duration.h"
#include "kdl_kinematics.hpp"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <sstream>
#include <cmath>
#include <complex>

using namespace Eigen;

void fifth_polinomials(MatrixXd &T, MatrixXd &q, MatrixXd &qd, MatrixXd &qdd, float ti, float tf, float qi, float dqi, float ddqi, float qf, float dqf, float ddqf, float Ts)
{

  MatrixXd H(6, 6);
  H << 1, ti, pow(ti, 2), pow(ti, 3), pow(ti, 4), pow(ti, 5),
      0, 1, 2 * ti, 3 * pow(ti, 2), 4 * pow(ti, 3), 5 * pow(ti, 4),
      0, 0, 2, 6 * ti, 12 * pow(ti, 2), 20 * pow(ti, 3),
      1, tf, pow(tf, 2), pow(tf, 3), pow(tf, 4), pow(tf, 5),
      0, 1, 2 * tf, 3 * pow(tf, 2), 4 * pow(tf, 3), 5 * pow(tf, 4),
      0, 0, 2, 6 * tf, 12 * pow(tf, 2), 20 * pow(tf, 3);

  MatrixXd Q(6, 1);
  Q << qi,
      dqi,
      ddqi,
      qf,
      dqf,
      ddqf;

  MatrixXd a = H.inverse() * (Q);

  MatrixXd a0 = a.row(0);
  MatrixXd a1 = a.row(1);
  MatrixXd a2 = a.row(2);
  MatrixXd a3 = a.row(3);
  MatrixXd a4 = a.row(4);
  MatrixXd a5 = a.row(5);

  int length = (int)floor((tf - ti) / Ts);
  int count = 0;
  for (float i = ti; i < length; i += Ts)
  {
    if (count >= length)
    {
      break;
    }
    T(0, count) = i;
    count++;
  }

  std::cout << "A coeff fifth polynomials: " << a0.coeff(0, 0) << " " << a1.coeff(0, 0) << " " << a2.coeff(0, 0) << " " << a3.coeff(0, 0) << " " << a4.coeff(0, 0) << " " << a5.coeff(0, 0) << "\n"
            << std::endl;
  for (int i = 0; i < length; i++)
  {

    q(0, i) = a5.coeff(0, 0) * pow(T.coeff(0, i), 5) + a4.coeff(0, 0) * pow(T.coeff(0, i), 4) + a3.coeff(0, 0) * pow(T.coeff(0, i), 3) + a2.coeff(0, 0) * pow(T.coeff(0, 0), 2) + a1.coeff(0, 0) * pow(T.coeff(0, i), 1) + a0.coeff(0, 0);
    qd(0, i) = 5 * a5.coeff(0, 0) * pow(T.coeff(0, i), 4) + 4 * a4.coeff(0, 0) * pow(T.coeff(0, i), 3) + 3 * a3.coeff(0, 0) * pow(T.coeff(0, i), 2) + 2 * a2.coeff(0, 0) * pow(T.coeff(0, i), 1) + a1.coeff(0, 0);
    qdd(0, i) = 20 * a5.coeff(0, 0) * pow(T.coeff(0, i), 3) + 12 * a4.coeff(0, 0) * pow(T.coeff(0, i), 2) + 6 * a3.coeff(0, 0) * pow(T.coeff(0, i), 1) + 2 * a2.coeff(0, 0);
  }
}

double sign_func(double x)
{
  if (x > 0)
  {
    return +1.0;
  }
  else if (x == 0)
  {
    return 0.0;
  }
  else
  {
    return -1.0;
  }
}

double vecangle(Vector3d &v1, Vector3d &v2, Vector3d &normal)
{

  double toRad = 2 * 3.14 / 360;
  Vector3d xprod = v1.cross(v2);

  double a = xprod.dot(normal);
  double s = sign_func(a);
  double c = s * (xprod.norm());
  return atan2(c, v1.dot(v2));

  // Conversion radian to degrees
  // radians = atan(y/x)
  // degrees = radians * (180.0/3.141592653589793238463)
}

int circular_length(MatrixXd &pi, MatrixXd &pf, float Ts, MatrixXd &c)
{

  MatrixXd n = (pi - c);
  MatrixXd rho(1, 1);
  rho(0, 0) = n.norm();
  Vector3d x = c - pf;
  Vector3d y = c - pi;
  Vector3d r = x.cross(y);
  double arc_angle = vecangle(x, y, r);
  double p_length = rho.coeff(0, 0) * arc_angle;

  int length = (int)floor(p_length / Ts);

  return length;
}

void circular_motion(MatrixXd &T, MatrixXd &p, MatrixXd &dp, MatrixXd &ddp, MatrixXd &pi, MatrixXd &pf, float Ts, MatrixXd &c, int length)
{

  MatrixXd n = (pi - c);

  MatrixXd rho(1, 1);
  rho(0, 0) = n.norm();
  Vector3d x = c - pf;
  Vector3d y = c - pi;
  Vector3d r = x.cross(y);

  MatrixXd s(1, length);
  int count = 0;
  for (float i = 0; i < length; i++)
  {
    if (count >= length)
    {
      break;
    }
    s(0, count) = i*Ts;
    count++;
  }

  std::cout << "s "<< s << std::endl;

  MatrixXd p_prime(3, length);
  // p_prime = [rho*cos((s/rho));rho*sin((s/rho));zeros(1,length(s))];
        
  MatrixXd zero(1, 1);
  zero << 0;
  p_prime.row(0) = (rho.coeff(0, 0) * ((s / rho.coeff(0, 0)).array().cos()));
  p_prime.row(1) = (rho.coeff(0, 0) * ((s / rho.coeff(0, 0)).array().sin()));
  for (int i = 0; i < length; i++)
  {
    p_prime(2, i) = zero.coeff(0, 0);
  }

  std::cout << "p_prime:" << p_prime << std::endl;
  MatrixXd R(3, 3);

  R.col(0) = (pi - c) / rho.coeff(0, 0);
  R.col(2) = r / (r.norm());
  Vector3d x1 = R.col(0);
  Vector3d z = R.col(2);
  R.col(1) = x1.cross(z);

  MatrixXd tmp(3, length);
  MatrixXd tmp1(3, length);
  MatrixXd tmp2(3, length);

  tmp.row(0) = -((s / rho.coeff(0, 0)).array().sin());
  tmp.row(1) = ((s / rho.coeff(0, 0)).array().cos());
  tmp.row(2) = p_prime.row(2);

  tmp1.row(0) = -((1 / rho.coeff(0, 0)) * (s / rho.coeff(0, 0)).array().cos());
  tmp1.row(1) = -((1 / rho.coeff(0, 0)) * (s / rho.coeff(0, 0)).array().sin());
  tmp1.row(2) = p_prime.row(2);

  // tmp.row(0)  = ((s/rho.coeff(0,0)).array().sin())/pow(rho.coeff(0,0),2);
  // tmp.row(1) = ((s/rho.coeff(0,0)).array().cos())/pow(rho.coeff(0,0),2);
  // tmp.row(2) = p_prime.row(2);

  for (int i = 0; i < length; i++)
  {
    p.col(i) = c + R * p_prime.col(i);
    dp.col(i) = R * tmp.col(i);
    ddp.col(i) = R * tmp1.col(i);
  }
}

void linear_motion(MatrixXd &T, MatrixXd &p, MatrixXd &dp, MatrixXd &ddp, MatrixXd &s, MatrixXd &pi, MatrixXd &pf, float Ts, int length1)
{

  MatrixXd support = pf - pi;
  double end = support.norm();

  MatrixXd zero(3, 1);
  zero << 0,
      0,
      0;

  for (int i = 0; i < length1; i++)
  {

    p.col(i) = pi + s.coeff(0, i) * (support / end);
    dp.col(i) = support / end;
    ddp.col(i) = zero;
  }
}

void circular_tilde(MatrixXd &T, MatrixXd &p_tilde, MatrixXd &dp_tilde, MatrixXd &ddp_tilde, MatrixXd &pi, MatrixXd &pf, MatrixXd &c, float ti, float tf, float Ts, int length_qf)
{

  MatrixXd n = (pi - c);

  MatrixXd rho(1, 1);
  rho(0, 0) = n.norm();
  Vector3d x = c - pf;
  Vector3d y = c - pi;
  Vector3d r = x.cross(y);

  float qi = 0;
  float dqi = 0;
  float ddqi = 0;
  float qf = (float)length_qf;
  float dqf = 0;
  float ddqf = 0;

  int length = (int)floor((tf - ti) / Ts);
  MatrixXd s(1, length);
  MatrixXd sd(1, length);
  MatrixXd sdd(1, length);

  fifth_polinomials(T, s, sd, sdd, ti, tf, qi, dqi, ddqi, qf, dqf, ddqf, Ts);

  MatrixXd p_prime(3, length);

  MatrixXd zero(1, 1);
  zero << 0;
  p_prime.row(0) = (rho.coeff(0, 0) * ((s / rho.coeff(0, 0)).array().cos()));
  p_prime.row(1) = (rho.coeff(0, 0) * ((s / rho.coeff(0, 0)).array().sin()));
  for (int i = 0; i < length; i++)
  {
    p_prime(2, i) = zero.coeff(0, 0);
  }

  MatrixXd R(3, 3);
  R.col(0) = (pi - c) / rho.coeff(0, 0);
  R.col(2) = r / (r.norm());
  Vector3d x1 = R.col(0);
  Vector3d z = R.col(2);
  R.col(1) = x1.cross(z);

  MatrixXd tmp(3, length);
  MatrixXd tmp1(3, length);
  //MatrixXd tmp2(3,length);

  MatrixXd tmp_1 = ((s / rho.coeff(0, 0)).array().sin());
  MatrixXd tmp_2 = ((s / rho.coeff(0, 0)).array().cos());

  for (int i = 0; i < length; i++)
  {

    tmp(0, i) = -tmp_1.coeff(0, i) * sd.coeff(0, i);
    tmp(1, i) = tmp_2.coeff(0, i) * sd.coeff(0, i);

    // tmp1.row(0)  = -((1/rho.coeff(0,0))*(s/rho.coeff(0,0)).array().cos());
    // tmp1.row(1) = -((1/rho.coeff(0,0))*(s/rho.coeff(0,0)).array().sin());
  }

  tmp.row(2) = p_prime.row(2);
  tmp1.row(2) = p_prime.row(2);

  // tmp.row(0)  = ((s/rho.coeff(0,0)).array().sin())/pow(rho.coeff(0,0),2);
  // tmp.row(1) = ((s/rho.coeff(0,0)).array().cos())/pow(rho.coeff(0,0),2);
  // tmp.row(2) = p_prime.row(2);

  for (int i = 0; i < length; i++)
  {
    p_tilde.col(i) = c + R * p_prime.col(i);
    dp_tilde.col(i) = R * tmp.col(i);
    ddp_tilde.col(i) = R * tmp1.col(i);
  }
}

void linear_tilde(MatrixXd &T, MatrixXd &p_tilde, MatrixXd &dp_tilde, MatrixXd &ddp_tilde, MatrixXd &pi, MatrixXd &pf, float ti, float tf, float Ts)
{

  float qi = 0;
  float dqi = 0;
  float ddqi = 0;
  MatrixXd support = pf - pi;
  float qf = (float)support.norm();
  float dqf = 0;
  float ddqf = 0;

  int length = (int)floor((tf - ti) / Ts);
  MatrixXd s(1, length);
  MatrixXd sd(1, length);
  MatrixXd sdd(1, length);

  fifth_polinomials(T, s, sd, sdd, ti, tf, qi, dqi, ddqi, qf, dqf, ddqf, Ts);

  for (int i = 0; i < length; i++)
  {
    p_tilde.col(i) = pi + (support / support.norm()) * s.col(i);
    dp_tilde.col(i) = pi + ((support) / support.norm()) * sd.col(i);
    ddp_tilde.col(i) = pi + ((support) / support.norm()) * sdd.col(i);
  }
}

void frenet_frame(MatrixXd &p, MatrixXd &dp, MatrixXd &ddp, MatrixXd &o_EE_t, MatrixXd &o_EE_n, MatrixXd &o_EE_b, MatrixXd &PHI_i, MatrixXd &PHI_f, int length)
{

  for (int column = 0; column < 2; column++)
  {

    int index = 0;
    if (column == 1)
    {
      index = length - 1;
    }

    MatrixXd n = (ddp.col(index)) / (ddp.col(index).norm());
    o_EE_t.col(column) = dp.col(index);
    o_EE_n.col(column) = n.col(0);

    Vector3d x = o_EE_t.col(column);
    Vector3d y = o_EE_n.col(column);
    MatrixXd b = x.cross(y);

    o_EE_b.col(column) = b.col(0);
  }

  MatrixXd R(3, 3);

  R.col(0) = o_EE_t.col(0);
  R.col(1) = o_EE_b.col(0);
  R.col(2) = o_EE_n.col(0);

  std::cout << "R" << R << std::endl;

  PHI_i(0, 0) = atan2(sqrt(pow(R.coeff(0, 2), 2) + pow(R.coeff(1, 2), 2)), R.coeff(2, 2));
  PHI_i(1, 0) = atan2(R.coeff(1, 2), R.coeff(0, 2));
  PHI_i(2, 0) = atan2(R.coeff(2, 1), -R.coeff(2, 0));

  R.col(0) = o_EE_t.col(1);
  R.col(1) = o_EE_b.col(1);
  R.col(2) = o_EE_n.col(1);

  PHI_f(0, 0) = atan2(sqrt(pow(R.coeff(0, 2), 2) + pow(R.coeff(1, 2), 2)), R.coeff(2, 2));
  PHI_f(1, 0) = atan2(R.coeff(1, 2), R.coeff(0, 2));
  PHI_f(2, 0) = atan2(R.coeff(2, 1), -R.coeff(2, 0));
}

void EE_orientation(MatrixXd &T, MatrixXd &PHI_i, MatrixXd &PHI_f, MatrixXd &o_tilde, MatrixXd &do_tilde, MatrixXd &ddo_tilde, MatrixXd &pi, MatrixXd &pf, float ti, float tf, float Ts)
{

  float qi = 0;
  float dqi = 0;
  float ddqi = 0;
  MatrixXd support = pf - pi;
  float qf = support.norm();
  float dqf = 0;
  float ddqf = 0;

  int length = (int)floor((tf - ti) / Ts);
  MatrixXd s(1, length);
  MatrixXd sd(1, length);
  MatrixXd sdd(1, length);

  fifth_polinomials(T, s, sd, sdd, ti, tf, qi, dqi, ddqi, qf, dqf, ddqf, Ts);

  MatrixXd support1 = PHI_f - PHI_i;
  MatrixXd l = support1 / support1.norm();
  for (int i = 0; i < length; i++)
  {
    o_tilde.col(i) = pi + l * s.col(i);
    do_tilde.col(i) = l * sd.col(i);
    ddo_tilde.col(i) = l * sdd.col(i);
  }
}

/**
 * This tutorial demonstrMatrix3f m;
ates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  // ros::init(argc, argv, "talker");
  // ros::NodeHandle n;

  // RobotArm ra(n);
  // ra.IKinematics(0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);

  // Parameters initialization
  float ti = 0;
  float tf = 4;
  //float qi = 1;
  //float dqi = 1;
  //float ddqi = 1;
  //float qf = 1;
  //float dqf = 1;
  //float ddqf = 1;
  float Ts = 0.1f;

  // Matrix q,qd,qdd initialization
  int length = (int)floor((tf - ti) / Ts);
  MatrixXd T(1, length);
  MatrixXd q(1, length);
  MatrixXd qd(1, length);
  MatrixXd qdd(1, length);

  //fifth_polinomials(T,q,qd,qdd,ti, tf, qi, dqi, ddqi, qf, dqf, ddqf, Ts);

  // Position in operationalspace with TIMING LAW
  MatrixXd p_tilde(3, length);
  MatrixXd dp_tilde(3, length);
  MatrixXd ddp_tilde(3, length);
  MatrixXd pi(3, 1);
  MatrixXd pf(3, 1);
  MatrixXd c(3, 1);
  c << 1,
      1,
      1;
  pi << 3,
      1,
      1;

  pf << 1,
      3,
      1;

  linear_tilde(T, p_tilde, dp_tilde, ddp_tilde, pi, pf, ti, tf, Ts);
  //std::cout<< "Linear_tilde: " << p_tilde << "\n" << std::endl;

  int length2 = circular_length(pi, pf, Ts, c);
  MatrixXd p_tilde1(3, length);
  MatrixXd dp_tilde1(3, length);
  MatrixXd ddp_tilde1(3, length);

  circular_tilde(T, p_tilde1, dp_tilde1, ddp_tilde1, pi, pf, c, ti, tf, Ts, length2);
  //std::cout << "Circular_tilde: " << dp_tilde1.row(0) << "\n" << std::endl;

  // linear motion - Position in operational space without TIMING LAW
  MatrixXd support = pf - pi;
  double end = support.norm();

  int length1 = (int)floor(end / Ts);
  MatrixXd s(1, length1);
  int count = 0;
  for (float i = ti; i < length1; i++)
  {
    if (count >= length1)
    {
      break;
    }
    s(0, count) = i;
    count++;
  }

  MatrixXd p(3, length1);
  MatrixXd dp(3, length1);
  MatrixXd ddp(3, length1);
  //linear_motion(T, p, dp, ddp, s, pi, pf, Ts, length1);
  std::cout << "Linear_motion: " << p << "\n" << std::endl;
  //std::cout << s.cols() << " " << p.cols() << std::endl;

  MatrixXd p1(3, length2);
  MatrixXd dp1(3, length2);
  MatrixXd ddp1(3, length2);
  circular_motion(T, p1, dp1, ddp1, pi, pf, Ts, c, length2);
  std::cout << "Circular_motion: " << p1 << "\n"<< std::endl;
  // Frenet Frame to find the orientation of the end-effector

  MatrixXd o_EE_t(3, 2);
  MatrixXd o_EE_n(3, 2);
  MatrixXd o_EE_b(3, 2);
  MatrixXd PHI_i(3, 1);
  MatrixXd PHI_f(3, 1);

  //frenet_frame(p, dp, ddp, o_EE_t, o_EE_n, o_EE_b, PHI_i, PHI_f, length1);
  frenet_frame(p1, dp1, ddp1, o_EE_t, o_EE_n, o_EE_b, PHI_i, PHI_f, length1);
  std::cout  << "PHI_i: " << PHI_i<< std::endl;

  // EE_orientation with the TIMING LAW
  MatrixXd o_tilde(3, length);
  MatrixXd do_tilde(3, length);
  MatrixXd ddo_tilde(3, length);
  EE_orientation(T, PHI_i, PHI_f, o_tilde, do_tilde, ddo_tilde, pi, pf, ti, tf, Ts);
  std::cout << "o_tilde" << o_tilde << std::endl;
  // Put the 3d coordinate togheter
  MatrixXd dataPosition(6, length);
  MatrixXd dataVelocities(6, length);
  MatrixXd dataAcceleration(6, length);

  dataPosition.block(0, 0, 3, length) = p_tilde.block(0, 0, 3, length);
  dataPosition.block(3, 0, 3, length) = o_tilde.block(0, 0, 3, length);

  dataVelocities.block(0, 0, 3, length) = dp_tilde.block(0, 0, 3, length);
  dataVelocities.block(3, 0, 3, length) = do_tilde.block(0, 0, 3, length);

  dataAcceleration.block(0, 0, 3, length) = ddp_tilde.block(0, 0, 3, length);
  dataAcceleration.block(3, 0, 3, length) = ddo_tilde.block(0, 0, 3, length);

  //std::cout << dataAcceleration << std::endl;

  // Ros node initialization
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<trajectory_msgs::JointTrajectory>("/robot/arm/pos_traj_controller/command", 1);

  ros::Rate loop_rate(1);
  int back = 0;
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  while (ros::ok())
  {
    back = 1 - back;
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    trajectory_msgs::JointTrajectory msg;
    msg.joint_names = {
        "robot_arm_elbow_joint",
        "robot_arm_shoulder_lift_joint",
        "robot_arm_shoulder_pan_joint",
        "robot_arm_wrist_1_joint",
        "robot_arm_wrist_2_joint",
        "robot_arm_wrist_3_joint",
        // "robot_wsg50_finger_left_joint",
        // "robot_wsg50_finger_right_joint",
    };

    std::vector<trajectory_msgs::JointTrajectoryPoint> points;
    for (int i = 0; i < 10; i++)
    {
      trajectory_msgs::JointTrajectoryPoint point;
      int val = i;
      /*if(back){
          val = 10-i;
        }*/
      //float p = val*0.5;
      float p = 0.02;
      point.positions = {p, p};     //{p,5,5,p,p,p,p,p};
      point.velocities = {p, p};    //{0,0,0,0,0,0,0,0};
      point.accelerations = {p, p}; //{0,0,0,0,0,0,0,0};
      point.time_from_start = ros::Duration(i);
      points.push_back(point);
    }

    msg.points = points;

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);
    //ROS_INFO("%s", msg.points.c_str());
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}