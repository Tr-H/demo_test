#include <stdlib.h>
#include "ros/ros.h"
#include "demo_test/pos_data.h"
#include "demo_test/pos_write_data2.h"
#include "demo_test/pos_status.h"
#include "demo_test/attitude_feedback.h"
#include "demo_test/velocity_feedback.h"
#include "demo_test/body_attitude.h"
#include "demo_test/position_setpoint.h"
#include <string.h>
#include <math.h>
#include <cstring>
#include <boost/thread.hpp>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <termios.h>
#include <stdlib.h>
#include <time.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <demo_test/demo_code.h>
#include <std_msgs/Float64.h>
#include "checksum.h"
std_msgs::Float64 global_att_throttle;
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
struct Q_att
{
    float q1;
    float q2;
    float q3;
    float q4;
};
struct Q_att from_euler(float roll, float pitch, float yaw) 
{
    double cosPhi_2 = cos(double(roll) / 2.0);
    double sinPhi_2 = sin(double(roll) / 2.0);
    double cosTheta_2 = cos(double(pitch) / 2.0);
    double sinTheta_2 = sin(double(pitch) / 2.0);
    double cosPsi_2 = cos(double(yaw) / 2.0);
    double sinPsi_2 = sin(double(yaw) / 2.0);
    struct Q_att Q_local;
    /* operations executed in double to avoid loss of precision through
     * consecutive multiplications. Result stored as float.
     */
    Q_local.q1 = static_cast<float>(cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2);
    Q_local.q2 = static_cast<float>(sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2);
    Q_local.q3 = static_cast<float>(cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2);
    Q_local.q4 = static_cast<float>(cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2);
    return Q_local;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavros_att_test");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);
    //ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //       ("mavros/setpoint_position/local", 10);
    ros::Publisher local_att_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_attitude/attitude", 10);
    ros::Publisher local_thrust_pub = nh.advertise<std_msgs::Float64>
        ("mavros/setpoint_attitude/att_throttle", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    struct Q_att Q_local_sp;
    float pitch_set = 0.0f;
    float roll_set = 0.0f;
    float yaw_set = 0.0f;
    Q_local_sp = from_euler(roll_set,pitch_set,yaw_set);
    geometry_msgs::PoseStamped att_sp;
    att_sp.pose.orientation.x = Q_local_sp.q2;
    att_sp.pose.orientation.y = Q_local_sp.q3;
    att_sp.pose.orientation.z = Q_local_sp.q4;
    att_sp.pose.orientation.w = Q_local_sp.q1;
    global_att_throttle.data = 0.3f;
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        ros::spinOnce();
        local_att_pub.publish(att_sp); 
        local_thrust_pub.publish(global_att_throttle);
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        ROS_INFO("p: %.2f r: %.2f y: %.2f th: %.2f",roll_set,pitch_set,yaw_set,global_att_throttle.data);
        Q_local_sp = from_euler(roll_set,pitch_set,yaw_set);
        att_sp.pose.orientation.x = Q_local_sp.q2;
        att_sp.pose.orientation.y = Q_local_sp.q3;
        att_sp.pose.orientation.z = Q_local_sp.q4;
        att_sp.pose.orientation.w = Q_local_sp.q1;
        global_att_throttle.data = 0.5f;
        local_att_pub.publish(att_sp); 
        local_thrust_pub.publish(global_att_throttle);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
