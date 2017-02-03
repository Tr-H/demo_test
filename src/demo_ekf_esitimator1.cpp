#include <stdlib.h>
#include "ros/ros.h"
#include "demo_test/pos_data.h"
#include "demo_test/ekf_status.h"
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
using namespace Eigen;
using namespace Eigen::internal;  
using namespace Eigen::Architecture; 
geometry_msgs::PoseStamped time_status;
boost::mutex pos_mutex;
demo_test::pos_data xyz;
ros::Publisher ekf_status_pub;
demo_test::ekf_status _ekf_status;
MatrixXf F(9,9);
MatrixXf G(9,3);
MatrixXf Q(3,3);
MatrixXf R(3,3);
MatrixXf Z_real(3,1);
MatrixXf X_pre(9,1);
MatrixXf X_ekf(9,1);
MatrixXf P_pre(9,9);
MatrixXf P_ekf(9,9);
MatrixXf V_n(3,1);
MatrixXf S(3,3);
MatrixXf H(3,9);
MatrixXf K(9,3);
MatrixXf I_9(9,9);//defined in main
bool status_need_update = false;
bool status_init = false;
bool have_update = false;
	float t1;
	float t2;
struct timespec time2={0, 0};
void udpcallback(const demo_test::pos_data& pos)
{
   //ROS_INFO_ONCE("|UAV%d| :: [ MOCAP ] DATE RECEIVED !!",global_id);
    boost::mutex::scoped_lock lock(pos_mutex);
    memcpy(&xyz, &pos, sizeof(pos));
    Z_real << xyz.pos[0], xyz.pos[1], xyz.pos[2];
    	if(!status_init)
    	{	
    		//init
    		X_pre << xyz.pos[0],0,0,xyz.pos[1],0,0,xyz.pos[2],0,0;
    		X_ekf = X_pre;
    		status_init = true;
    	}
    V_n = Z_real - H*X_ekf;
    status_need_update = true;
    lock.unlock();
}

void predict()
{
	//TODO : PREDICT
	float T;

	bool time_init = false;
	ros::Rate predict_loop_rate(80);
	while(ros::ok())
	{	
		clock_gettime(CLOCK_MONOTONIC, &time2);
		t2 = (float)(time2.tv_sec + time2.tv_nsec*0.000000001);
		if (time_init)
		{T = t2 - t1;}
		else
		{T = 0;}
		F << 	1, T, pow(T,2)/2,0,0,0,0,0,0,
				0, 1, T,0,0,0,0,0,0,
				0, 0, 1,0,0,0,0,0,0,
				0, 0, 0, 1, T, pow(T,2)/2,0,0,0,
				0, 0, 0, 0, 1, T,0,0,0,
				0, 0, 0, 0, 0, 1,0,0,0,
				0, 0, 0, 0, 0, 0, 1, T, pow(T,2)/2,
				0, 0, 0, 0, 0, 0, 0, 1, T,
				0, 0, 0, 0, 0, 0, 0, 0, 1;
		G << 	2*pow(T,3)/6, 0,	0,
				0.01*pow(T,2)/2, 0,	0,
				0.0003*T,	0,	0,
				0,	2*pow(T,3)/6, 0,
				0,	0.01*pow(T,2)/2, 0,
				0,	0.0003*T,	0,
				0,	0,	2*pow(T,3)/6,//1.5
				0,	0,	0.01*pow(T,2)/2,//0.3 0.6
				0,	0,	0.0003*T;//0.015 0.1
		if(status_init && time_init)
    	{	
    		if (have_update)
    		{
				X_pre = F*X_ekf;
				P_pre = F*P_ekf*F.transpose() + G*Q*G.transpose();
				have_update = false;
			}
			else
			{
				X_pre = F*X_pre;
				P_pre = F*P_pre*F.transpose() + G*Q*G.transpose();
			}
			ROS_INFO_ONCE("ekf_test1 predict part RUNING!!");

		}
		else
		{
			X_pre = X_ekf;
			P_pre = P_ekf;
		}
		_ekf_status.header.stamp = ros::Time::now();
		_ekf_status.pos[0] = X_pre(0,0);
		_ekf_status.pos[1] = X_pre(3,0);
		_ekf_status.pos[2] = X_pre(6,0);
		_ekf_status.velocity[0] = X_pre(1,0);
		_ekf_status.velocity[1] = X_pre(4,0);
		_ekf_status.velocity[2] = X_pre(7,0);
		_ekf_status.accel[0] = X_pre(2,0);
		_ekf_status.accel[1] = X_pre(5,0);
		_ekf_status.accel[2] = X_pre(8,0);
		ekf_status_pub.publish(_ekf_status);
		t1 = t2;
		time_init = true;
		predict_loop_rate.sleep();
	}
}

void update()
{
	//TODO : update
	ros::Rate update_loop_rate(10);
	while(ros::ok())
	{	
		if (status_need_update && status_init)
		{	
			ROS_INFO_ONCE("ekf_test1 update part RUNING!!");
			S = H*P_pre*H.transpose() + R;
			K = P_pre*H.transpose()*S.inverse();
			X_ekf = X_pre + K*V_n;
			P_ekf = (I_9 - K*H)*P_pre;
			t1 = (float)(time2.tv_sec + time2.tv_nsec*0.000000001);
			have_update = true;
			status_need_update = false;
		}
		update_loop_rate.sleep();	
	}

}

int main(int argc, char **argv)
{	
	ros::init(argc,argv,"ekf_test1");
	ros::NodeHandle nh;
	//init part
		I_9 = MatrixXf::Identity(9,9);
		X_pre = MatrixXf::Zero(9,1);
		X_ekf = MatrixXf::Zero(9,1);
		 F = MatrixXf::Zero(9,9);
         G = MatrixXf::Zero(9,3);
		 Z_real = MatrixXf::Zero(3,1);
	     V_n = MatrixXf::Zero(3,1);
		 S = MatrixXf::Zero(3,3);
	     K = MatrixXf::Zero(9,3);

		Q << 200000, 0, 0,
		     0, 200000, 0,
		     0, 0, 200000;

		R << 0.00003, 0, 0,
			 0, 0.00003, 0,
			 0, 0, 0.00003;//0.00003

		H << 1,0,0,0,0,0,0,0,0,
			 0,0,0,1,0,0,0,0,0,
			 0,0,0,0,0,0,1,0,0;

		// P_ekf<< 198.013131962190,19.8011344363992,0.990049840588496,0,0,0,0,0,0,
		// 	    19.8011344363992,2.98009141426629,0.199004022138792,0,0,0,0,0,0,
		// 	    0.990049840588497,0.199004022138792,0.0199501390187370,0,0,0,0,0,0,
		// 	    0,0,0,198.013131962190,19.8011344363992,0.990049840588496,0,0,0,
		// 	    0,0,0,19.8011344363992,2.98009141426629,0.199004022138792,0,0,0,
		// 	    0,0,0,0.990049840588497,0.199004022138792,0.0199501390187370,0,0,0,
		// 	    0,0,0,0,0,0,198.013131962190,98.011344363992,0.990049840588496,
		// 		0,0,0,0,0,0,19.8011344363992,2.98009141426627,0.199004022138790,
		// 		0,0,0,0,0,0,0.990049840588496,0.199004022138790,0.0199501390187369; 
			 P_ekf = MatrixXf::Identity(9,9)*10;
		P_pre = P_ekf;
	//init end
	ros::Subscriber sub = nh.subscribe("/demo_udp", 10, &udpcallback);
	ekf_status_pub = nh.advertise<demo_test::ekf_status>("ekf_test1/ekf_status",10);
	boost::thread thrd(&predict);
	boost::thread thrd2(&update);
	ros::AsyncSpinner s(4);
	s.start();
	 ros::Rate loop_rate(10);
	// // m << (Matrix3f() << 1, 2, 3, 4, 5, 6, 7, 8, 9).finished();
	// m << 1, 2, 3,
 //                      4, 5, 6,
 //                      7, 8, 9;
	while(ros::ok())
	{
		ROS_INFO_ONCE("ekf_test1 RUNING!!");
		cout << "X_pre" <<endl;
		cout << X_pre.transpose() << endl;
		cout << "p_ekf" <<endl;
		cout << P_ekf << endl;
		cout << "H" <<endl;
		cout << H << endl;
		cout << "K" <<endl;
		cout << K << endl;
		cout << "V_n" <<endl;
		cout << V_n << endl;
		cout << "S" <<endl;
		cout << S << endl;
		loop_rate.sleep();
	}
	return 0;
}