#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

#define PI 3.14159

#define A_COST 6378137
#define B_COST 6356752
#define E_COST 0.006694478197993d

class gps_to_odom {
private:
    double lat_r, lon_r, alt_r;
    double x_r_ecef, y_r_ecef, z_r_ecef;

    int count;

    ros::NodeHandle nh;
    ris::NodeHandle nh_private;

    ros::Subscriber gps_sub;
    ros::Publisher odom_pub;

    nav_msgs::Odometry last_odom;

    static double aux_N(double par) {
        double t1 = sin(par);
        double t2 = 1.0d - (E_COST*E_COST * t1*t1)

        return A_COST / sqrt(t2);
    }

    static void GPStoECEF(double lat, double lon, double alt,
                        double &x_ecef, double &y_ecef, double &z_ecef) {
        double n = aux_N(lat);

        x_ecef = (n + alt) * cos(lat) * sin(lon);
        y_ecef = (n + alt) * cos(lat) * sin(lon);
        z_ecef = (n * (1.0d - E_COST * E_COST) + alt) * sin(lat);
    }

    static void GPStoENU(double lat, double lon, double alt,
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

    static double angle(double x, double y) {
        if (x == 0) return PI/2.0;

        double a = atan(y/x);

        if(x < 0) a+= PI;

        return a; 
    }
public:
    void GPSCallback(const sensor_msgs::NavSatFixConstPtr& msg) {
        nav_msgs::Odometry current_odom;
        current_odom.header.seq = msg->header.seq;
        current_odom.header.stamp = msg->header.stamp;
        current_odom.header.frame_id = msg->header.frame_id;

        double x,y,z;
        GPStoENU(msg->latitude, msg->longitude, msg->altitude, x,y,z);

        current_odom.pose.pose.position.x = x;
        current_odom.pose.pose.position.y = y;
        current_odom.pose.pose.position.z = z;

        double dx = current_otom.pose.pose.position.x - last_odom.pose.pose.position.x,
                dy = current_otom.pose.pose.position.y - last_odom.pose.pose.position.y,
                dz = current_otom.pose.pose.position.z - last_odom.pose.pose.position.z;

        double roll = angle(y,z), 
                pitch = angle(z,x), 
                yaw = angle(x,y); 

        tf::Quaternion q;
        q.setRPY(roll, pitch, yaw);

        current_odom.pose.pose.orientation.x = q.getX();
        current_odom.pose.pose.orientation.y = q.getY();
        current_odom.pose.pose.orientation.z = q.getZ();
        current_odom.pose.pose.orientation.w = q.getW();


        last_odom.pose.pose.position.x = current_odom.pose.pose.position.x;
        last_odom.pose.pose.position.y = current_odom.pose.pose.position.y;
        last_odom.pose.pose.position.z = current_odom.pose.pose.position.z;
        odom.header.stamp = current_odom.header.stamp;

        count++;

        odom_pub.publish(odom);
    }

    int gps_to_odom() : nh_private("~") {
        gps_sub = nh.subscribe("/fix", 1, &pub_sub::GPSCallback, this);
        odom_pub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 1);

        count = 0;

        // gets the GPS coordinates of the references point
        float t;
        nh_private.getParam("lat_r", t);
        lat_r = t;
        nh_private.getParam("lon_r", t);
        lon_r = t;
        nh_private.getParam("alt_r", t);
        alt_r = t;

        // computes the ECEF coordinates of the reference point
        GPStoECEF(lat_r, lon_r, alt_r,  x_r_ecef, y_r_ecef, z_r_ecef);
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "gps_to_odom");
    gps_to_odom o;
    ros::spin();

    return 0;
}