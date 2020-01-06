/*#include <ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
extern volatile long left_count;
extern volatile long right_count;



class Odometry
{
  private:
    long _currentLeftcount,_currentRightcount;
    long _previousLeftcount,_previousRightcount;
    int deltaLeft,deltaRight;
    double dt;
    double v_left,v_right,Vx,Vth;
    double x,y,th;
    double dist_per_count = 1;
    double wheel_axle_length = 1;
    
    ros::Time current_time,last_time;
    geometry_msgs::TransformStamped odom_trans;
    geometry_msgs::Quaternion odom_quat;
    ros::NodeHandle*  nh;
    tf::TransformBroadcaster _brdcst;
    nav_msgs::Odometry odom;
    ros::Publisher odom_pub;
    
  public:
    Odometry(ros::NodeHandle*  test) : odom_pub("odom",&odom){
      nh = test;
    }
    void set(){
      _brdcst.init(*nh);
      nh->advertise(odom_pub);
    }
    void compute_wheel_odom(){
      current_time = nh->now();
      _currentLeftcount = left_count; 
      _currentRightcount = right_count; 

      deltaLeft = _currentLeftcount - _previousLeftcount;
      deltaRight = _currentRightcount - _previousRightcount;
      dt = current_time.toSec() - last_time.toSec();
  
      v_left = (deltaLeft*dist_per_count)/dt;
      v_right = (deltaRight*dist_per_count)/dt; 

      Vx = (v_right + v_left)/2;
      Vth = (v_right - v_left)/wheel_axle_length;

      x += Vx*cos(th)*dt;
      y += Vx*sin(th)*dt;
      th = Vth*dt;
      odom_quat = tf::createQuaternionFromYaw(th);
    
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";
      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;
      _brdcst.sendTransform(odom_trans);

      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0;
      odom.pose.pose.orientation = odom_quat;
      odom_pub.publish(&odom);

      last_time = current_time;
      _previousLeftcount = _currentLeftcount;
      _previousRightcount = _currentRightcount;
    }

};*/
