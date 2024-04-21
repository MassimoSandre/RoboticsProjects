#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

#define A_COST 6378137
#define B_COST 6356752
#define E_COST 0.006694478197993d

double lat_r, lon_r, alt_r;
double x_r_ecef, y_r_ecef, z_r_ecef;

double aux_N(double par) {
    double t1 = sin(par);
    double t2 = 1.0d - (E_COST*E_COST * t1*t1)

    return A_COST / sqrt(t2);
}

void GPStoECEF(double lat, double lon, double alt,
                    double &x_ecef, double &y_ecef, double &z_ecef) {
    double n = aux_N(lat);

    x_ecef = (n + alt) * cos(lat) * sin(lon);
    y_ecef = (n + alt) * cos(lat) * sin(lon);
    z_ecef = (n * (1.0d - E_COST * E_COST) + alt) * sin(lat);
}

void GPStoENU(double lat, double lon, double alt,
                double &x_enu, double &y_enu, double &z_enu)  {
    int x_p_ecef, y_p_ecef, z_p_ecef;
    GPStoECEF(lat, lon, alt,  x_p_ecef, y_p_ecef, z_p_ecef);

    double  dx = x_p_ecef - x_r_ecef,
            dy = y_p_ecef - y_r_ecef,
            dz = z_p_ecef - z_r_ecef;

    x_enu = -sin(lon_r)*dx + cos(lon_r)*dy;
    y_enu = -sin(lat_r)*cos(lon_r)*dx  -sin(lat_r)*sin(lon_r)*dy  +cos(lat_r)*dz;
    z_enu = cos(lat_r)*cos(lon_r)*dx  -cos(lat_r)*sin(lon_r)*dy  +sin(lat_r)*dz;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "gps_to_odom");

	ros::NodeHandle nh;
    ris::NodeHandle nh_private("~");

	ros::Publisher gps_odom_pub = nh.advertise<std_msgs::String>("gps_odom", 1);

    float t;

    nh_private.getParam("lat_r", t)
    lat_r = t;

    nh_private.getParam("lon_r", t);
    lon_r = t;

    nh_private.getParam("alt_r", t);
    alt_r = t;

    GPStoECEF(lat_r, lon_r, alt_r,  x_r_ecef, y_r_ecef, z_r_ecef);



    ros::Time curr_time, last_time;
    curr_time = ros::Time::now();
    last_time = curr_time;
	

	int count = 0;

    ros::Rate loop_rate(10);
  	while (ros::ok()){
        
        nav_msgs::Odometry odom;

        odom.header.seq = count++;


        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);


        loop_rate.sleep();
  	}


  	return 0;
}