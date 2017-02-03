#include <stdlib.h>
#include "ros/ros.h"
#include "demo_test/pos_data.h"
#include "demo_test/ekf_leader.h"
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
#include "checksum.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <time.h>
using namespace std;
boost::mutex pos_mutex;
demo_test::pos_data xyz;
ros::Publisher leader_pub;
demo_test::ekf_leader _ekf_leader;
bool update_yet = false;
void udpcallback(const demo_test::pos_data& pos)
{
   //ROS_INFO_ONCE("|UAV%d| :: [ MOCAP ] DATE RECEIVED !!",global_id);
    boost::mutex::scoped_lock lock(pos_mutex);
    memcpy(&xyz, &pos, sizeof(pos));
    update_yet = true;
    lock.unlock();
}
int main(int argc, char **argv)
{	
	ros::init(argc,argv,"fakeleader_send1");
	ros::NodeHandle nh;
	bool san1 = true;
	bool san2 = true;
	bool san3 = true;
	leader_pub = nh.advertise<demo_test::ekf_leader>("mavros2/leader_topic/leader_topic",10);
	_ekf_leader.pos[0]= 0;//xyz.pos[0];
			_ekf_leader.pos[1]= 1;//xyz.pos[2];
			_ekf_leader.pos[2]= 2;//-xyz.pos[1];
			float x = 1.0f;
	ros::Rate loop_rate(20);
	while(ros::ok())
	{
		// if(update_yet)
		// {	
			_ekf_leader.header.stamp= ros::Time::now();//xyz.header.stamp;
				_ekf_leader.pos[0]=2/3.1415926*sin(x/40 * 3.1415926);
				_ekf_leader.pos[1]=2/3.1415926*cos(x/40 * 3.1415926);
				_ekf_leader.pos[2]= 2/3.1415926*cos(x/28 * 3.1415926);
			// if (_ekf_leader.pos[0]<=2.0f && san1)
			// {
			// 	_ekf_leader.pos[0]= _ekf_leader.pos[0]+0.05f;
			// }
			// else
			// {
			// 	_ekf_leader.pos[0]= _ekf_leader.pos[0]-0.05f;//sin(x/20);//_ekf_leader.pos[0]+0.01;//xyz.pos[0];
			// 	san1 = false;
			// 	if(_ekf_leader.pos[0]<= -2.0f)
			// 		{
			// 			san1 = true;
			// 		}
			// }
			// if (_ekf_leader.pos[1]<=2.0f && san2)
			// {
			// 	_ekf_leader.pos[1]= _ekf_leader.pos[1]+0.08f;//sin(x/20);//cos(x/20);//_ekf_leader.pos[1]+0.02;//xyz.pos[2];
			// }
			// else
			// {
			// 	_ekf_leader.pos[1]= _ekf_leader.pos[1]-0.08f;
			// 	san2 = false;
			// 	if(_ekf_leader.pos[1]<= -2.0f)
			// 		{
			// 			san2 = true;
			// 		}
			// }
			// if (_ekf_leader.pos[2]<=2.0f && san3)
			// {
			// 	_ekf_leader.pos[2]= _ekf_leader.pos[2]+0.1f;//sin(x/20);//sin(x/20)*sin(x/20);//_ekf_leader.pos[2]+0.03;//-xyz.pos[1];
			// }
			// else
			// {
			// 	_ekf_leader.pos[2]= _ekf_leader.pos[2]-0.1f;
			// 	san3 = false;
			// 	if(_ekf_leader.pos[2]<= -2.0f)
			// 		{
			// 			san3 = true;
			// 		}
			// }
			x = x+1.0f;
			leader_pub.publish(_ekf_leader);
			update_yet = false;
		// }
		// else
		// {
		// 	ROS_INFO_THROTTLE(2,"wait for data....");
		// }
		ROS_INFO_ONCE("fakeleader_send1 RUNING!!");
		loop_rate.sleep();
	}
	return 0;
}