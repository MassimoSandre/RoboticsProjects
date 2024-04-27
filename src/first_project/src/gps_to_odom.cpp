#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <math.h>
#include <tf/LinearMath/Quaternion.h>

#define PI 3.14159d
#define DEG_TO_RAD PI/180.0d


#define A_CONST 6378137.0d
#define B_CONST 6356752.0d

// E_SQR = 1 - (A/B)^2
#define E_SQR_CONST 0.006694478197993d

/**
 * gps_to_odom node that reads NavSatFix messages from /fix.
 * For each message received, it publishes a Odometry message representing the ENU location.
 * Requires LLA coordinates for the reference point provided as params in the node's namespace.
*/
class gps_to_odom {
private:
    // LLA coordinates of the reference point
    double lat_r, lon_r, alt_r;

    // ECEF coordinates of the reference point
    double X_r, Y_r, Z_r;

    // the previous heading of the robot, used when the robot stays still
    double last_heading;

    // node handles, private one used to access the node's param
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    // subscriber for the NatSatFix messages
    ros::Subscriber gps_sub;
    // publisher for the Odometry messages
    ros::Publisher odom_pub;

    // The last message sent (position is used to approximate the heading)
    nav_msgs::Odometry last_odom;

    /**
     * Computes the sin of an angle provided in degrees
     * 
     * @param angle a value in degrees representing an angle
     * @return the sin value of the provided angle 
    */
    static double sin_deg(double angle) {
        // a_rad = a_deg * PI / 180
        return sin(angle * DEG_TO_RAD);
    }

    /**
     * Computes the cos of an angle provided in degrees
     * 
     * @param angle a value in degrees representing an angle
     * @return the cos value of the provided angle 
    */
    static double cos_deg(double angle) {
        // a_rad = a_deg * PI / 180
        return cos(angle * DEG_TO_RAD);
    }

    /**
     * Converts LLA coordinates in ECEF coordinates
     * 
     * @param lat the latitude of the location whose coordinates need to be converted
     * @param lon the longitude of the location whose coordinates need to be converted
     * @param lat the altitude of the location whose coordinates need to be converted
     * @param x the container for the component of the ECEF point that lays on the X axis
     * @param y the container for the component of the ECEF point that lays on the Y axis
     * @param z the container for the component of the ECEF point that lays on the Z axis
    */
    void GPStoECEF(double lat, double lon, double alt,
                    double &x, double &y, double &z) {
        
        // applying the formulae to convert LLA coordinates to ECEF coordinates
        double t = sin_deg(lat);
        double N = A_CONST / sqrt(1.0d - E_SQR_CONST * t * t);

        x = (N + alt) * cos_deg(lat) * cos_deg(lon);
        y = (N + alt) * cos_deg(lat) * sin_deg(lon);
        z = (N * (1.0d - E_SQR_CONST) + alt) * sin_deg(lat);
    }

    /**
     * Converts LLA coordinates in ENU coordinates centered around the reference point
     * 
     * @param lat the latitude of the location whose coordinates need to be converted
     * @param lon the longitude of the location whose coordinates need to be converted
     * @param lat the altitude of the location whose coordinates need to be converted
     * @param e the container for the component of the ENU point that lays on the axis pointing East
     * @param n the container for the component of the ENU point that lays on the axis pointing North
     * @param u the container for the component of the ENU point that lays on the axis pointing Up
    */
    void GPStoENU(double lat, double lon, double alt, 
                    double &e, double &n, double &u) {

        // converting the LLA coordinates in ECEF coordinates
        double X_p, Y_p, Z_p;
        GPStoECEF(lat, lon, alt, X_p, Y_p, Z_p);

        // computing the subtraction of the reference point from the given point, both in ECEF coordinates
        double  dx = X_p - X_r,
                dy = Y_p - Y_r,
                dz = Z_p - Z_r;

        // applying the formulae to convert ECEF coordinates to ENU coordinates
        e = - sin_deg(lon_r)*dx + cos_deg(lon_r)*dy;
        n = - sin_deg(lat_r)*cos_deg(lon_r)*dx -sin_deg(lat_r)*sin_deg(lon_r)*dy + cos_deg(lat_r)*dz;
        u = + cos_deg(lat_r)*cos_deg(lon_r)*dx +cos_deg(lat_r)*sin_deg(lon_r)*dy + sin_deg(lat_r)*dz;
    }

    /**
     * Computes an approximation of the robot heading given the movement on the North and East axis.
     * If the robot hasn't moved, the last heading will be returned (which is set to zero at the beginning).
     * 
     * @param dEast the component on the East axis of the robot movement
     * @param dNorth the component on the North axis of the robot movement
     * @return an approximation of the robot heading
    */
    double computeHeading(double dEast, double dNorth) {
        // no movement detected => last_heading is the best approximation for the robot current heading
        if (dEast == 0 && dNorth == 0) {
            return last_heading;
        } 

        // computing the orientation of the robot movement
        // since the position is only updated 5 times a second
        // that value may diverge a little from the actual one
        return atan2(north,east);
    }

public:
    /**
     * Callback function triggered when a GPS message is received.
     * Converts the location from LLA to ENU coordinates and publish an
     * odometry message representing the robot location according to the GPS
     * 
     * @param msg a pointer to the message containing the GPS data
    */
    void GPSCallback(const sensor_msgs::NavSatFixConstPtr& msg) {
        // Copying the message information in the new message
        // (I decided to mantain the "old" message's timestamp because
        // that time is linked to the location and that location was
        // measured in that precise timestamp)
        nav_msgs::Odometry current_odom;
        current_odom.header.seq = msg->header.seq;
        current_odom.header.stamp = msg->header.stamp;

        // assigning gps_odom as frame id for the message
        std::string frame_id = "gps_odom";
        current_odom.header.frame_id = frame_id.c_str();

        // converting the LLA coordinates to ENU coordinates and
        // putting them in the new message
        double e,n,u;
        GPStoENU(msg->latitude, msg->longitude, msg->altitude, e,n,u);
        current_odom.pose.pose.position.x = e;
        current_odom.pose.pose.position.y = n;
        current_odom.pose.pose.position.z = u;

        // approximating the robot heading given the two most recent known positions
        double dEast = current_odom.pose.pose.position.x - last_odom.pose.pose.position.x,
                dNorth = current_odom.pose.pose.position.y - last_odom.pose.pose.position.y;
        last_heading = computeHeading(dEast, dNorth);

        // building the quaternion representing the robot orientation
        // (pitch not included in the computation)
        tf::Quaternion q;
        q.setRPY(0, 0, last_heading);
        current_odom.pose.pose.orientation.x = q.getX();
        current_odom.pose.pose.orientation.y = q.getY();
        current_odom.pose.pose.orientation.z = q.getZ();
        current_odom.pose.pose.orientation.w = q.getW();

        // updating the stored position with the last known position (i.e. the current one)
        last_odom.pose.pose.position.x = current_odom.pose.pose.position.x;
        last_odom.pose.pose.position.y = current_odom.pose.pose.position.y;
        last_odom.pose.pose.position.z = current_odom.pose.pose.position.z;

        // publishing the message that was just built
        odom_pub.publish(current_odom);    
    }

    /**
     * Builds the gps_to_odom node object.
     * Subscribes to the /fix topic and advertises the /gps_odom topic.
     * Resets the last_heading value.
     * Reads the reference point from the node's params.
    */
    gps_to_odom() : nh_private("~") {
        // Subscribing to the /fix topic, so that messages from the GPS can be handled
        gps_sub = nh.subscribe("/fix", 1, &gps_to_odom::GPSCallback, this);
        // Advertising the /gps_odom topic, used to publish the Odometry of the robot according to the GPS data
        odom_pub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 1);

        // resets the current robot heading
        last_heading = 0.0d;

        // gets the GPS coordinates of the references point
        this->nh_private.getParam("lat_r", lat_r);
        this->nh_private.getParam("lon_r", lon_r);  
        this->nh_private.getParam("alt_r", alt_r);
        
        // computes the ECEF coordinates of the reference point
        GPStoECEF(lat_r, lon_r, alt_r,  X_r, Y_r, Z_r);
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "gps_to_odom");
    gps_to_odom o;
    ros::spin();

    return 0;
}