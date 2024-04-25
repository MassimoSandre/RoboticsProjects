#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <math.h>
#include <tf/LinearMath/Quaternion.h>

#define PI 3.14159d

#define DEG_TO_RAD PI/180.0d

/*
#define A_COST 6378137.0d
#define B_COST 6356752.0d
#define E_SQR_COST 0.006694478197993d
*/

class gps_to_odom {
private:
    double lat_r, lon_r, alt_r;
    double x_r_ecef, y_r_ecef, z_r_ecef;

    double last_pitch, last_yaw;

    int count;

    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::Subscriber gps_sub;
    ros::Publisher odom_pub;

    nav_msgs::Odometry last_odom;

    double A_CONST = 6378137.0d, B_CONST = 6356752.314d, E_SQR_CONST;

    double aux_N(double par) {
        double t1 = sin(par);
        double t2 = 1.0d - (E_SQR_CONST * t1*t1);

        return A_CONST / sqrt(t2);
    }

    void GPStoECEF(double lat, double lon, double alt,
                        double &x_ecef, double &y_ecef, double &z_ecef) {
        double n = aux_N(lat);

        x_ecef = (n + alt) * cos(lat) * cos(lon);
        y_ecef = (n + alt) * cos(lat) * sin(lon);
        z_ecef = (n * (1.0d - E_SQR_CONST) + alt) * sin(lat);
    }

    void GPStoENU(double lat, double lon, double alt,
                    double &x_enu, double &y_enu, double &z_enu)  {
        double x_p_ecef, y_p_ecef, z_p_ecef;
        GPStoECEF(lat, lon, alt,  x_p_ecef, y_p_ecef, z_p_ecef);

        double  dx = x_p_ecef - x_r_ecef,
                dy = y_p_ecef - y_r_ecef,
                dz = z_p_ecef - z_r_ecef;

        x_enu = -sin(lon_r)*dx + cos(lon_r)*dy;
        y_enu = -sin(lat_r)*cos(lon_r)*dx  -sin(lat_r)*sin(lon_r)*dy  +cos(lat_r)*dz;
        z_enu = cos(lat_r)*cos(lon_r)*dx  +cos(lat_r)*sin(lon_r)*dy  +sin(lat_r)*dz;
    }

    static void orientation(double x, double y, double z, double last_pitch, double last_yaw, double &pitch, double &yaw) {
        if (x == 0 && y == 0) {
            if(z != 0) {
                pitch = PI/2.0d;
            }
            else {
                pitch = last_pitch;
            }
        } 
        else {
            pitch = atan2(z, sqrt(x*x + y*y));
        }

        if(x == 0) {
            if(y != 0) {
                yaw = PI/2.0d;
            }
            else {
                yaw = last_yaw;
            }
        }
        else {
            yaw = atan2(y,x);
        }
    }
public:
    void GPSCallback(const sensor_msgs::NavSatFixConstPtr& msg) {
        nav_msgs::Odometry current_odom;
        current_odom.header.seq = msg->header.seq;
        current_odom.header.stamp = msg->header.stamp;

        std::string frame_id = "gps_odom";
        current_odom.header.frame_id = frame_id.c_str();

        double x,y,z;
        GPStoENU(msg->latitude * DEG_TO_RAD, msg->longitude * DEG_TO_RAD, msg->altitude, x,y,z);

        current_odom.pose.pose.position.x = x;
        current_odom.pose.pose.position.y = y;
        current_odom.pose.pose.position.z = z;

        if (count > 0) {
            double dx = current_odom.pose.pose.position.x - last_odom.pose.pose.position.x,
                    dy = current_odom.pose.pose.position.y - last_odom.pose.pose.position.y,
                    dz = current_odom.pose.pose.position.z - last_odom.pose.pose.position.z;

            double pitch, yaw; 
            if(count > 1) {
                orientation(dx, dy, dz, last_pitch, last_yaw, pitch, yaw);
            }
            else {
                orientation(dx, dy, dz, 0, 0, pitch, yaw);
            }

            last_pitch = pitch;
            last_yaw = yaw;

            tf::Quaternion q;
            q.setRPY(0, pitch, yaw);
            

            current_odom.pose.pose.orientation.x = q.getX();
            current_odom.pose.pose.orientation.y = q.getY();
            current_odom.pose.pose.orientation.z = q.getZ();
            current_odom.pose.pose.orientation.w = q.getW();

            // ROS_INFO("received: east=%f, north=%f, up=%f \n orientation: x=%f,  y=%f,  z=%f, w=%f\n pitch=%f, yaw=%f\n",
            //     current_odom.pose.pose.position.x,
            //     current_odom.pose.pose.position.y,
            //     current_odom.pose.pose.position.z,
            //     current_odom.pose.pose.orientation.x,
            //     current_odom.pose.pose.orientation.y,
            //     current_odom.pose.pose.orientation.z,
            //     current_odom.pose.pose.orientation.w,
            //     pitch,
            //     yaw
            // );
        }
        else {
            current_odom.pose.pose.orientation.x = 0;
            current_odom.pose.pose.orientation.y = 0;
            current_odom.pose.pose.orientation.z = 0;
            current_odom.pose.pose.orientation.w = 1;
        }
        // else {
        //     ROS_INFO("FIRST received: east=%f, north=%f, up=%f \n orientation: x=%f,  y=%f,  z=%f, w=%f\n pitch=%f, yaw=%f\n",
        //         current_odom.pose.pose.position.x,
        //         current_odom.pose.pose.position.y,
        //         current_odom.pose.pose.position.z,
        //         0,
        //         0,
        //         0,
        //         0,
        //         0,
        //         0
        //     );
        // }


        last_odom.pose.pose.position.x = current_odom.pose.pose.position.x;
        last_odom.pose.pose.position.y = current_odom.pose.pose.position.y;
        last_odom.pose.pose.position.z = current_odom.pose.pose.position.z;
        last_odom.header.stamp = current_odom.header.stamp;

        count++;

        odom_pub.publish(current_odom);

        
              
    }

    gps_to_odom() : nh_private("~") {
        gps_sub = nh.subscribe("/fix", 1, &gps_to_odom::GPSCallback, this);
        odom_pub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 1);

        count = 0;

        double t = B_CONST/A_CONST;
        t *= t;
        E_SQR_CONST = 1 - t;

        // gets the GPS coordinates of the references point
        this->nh_private.getParam("lat_r", t);
        lat_r = t * DEG_TO_RAD;
        this->nh_private.getParam("lon_r", t);
        lon_r = t * DEG_TO_RAD;
        this->nh_private.getParam("alt_r", t);
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