#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <math.h>
#include <tf/LinearMath/Quaternion.h>

// Set ROTATE_REF_SYSTEM_TO_MATCH_WHEEL_ODOMETRY = false    to use the ENU reference system for the GPS odometry.
// Set ROTATE_REF_SYSTEM_TO_MATCH_WHEEL_ODOMETRY = true     to make the GPS odometry reference system "match" the wheel odometry one
//                                                          The new reference system is not ENU anymore. Used to check that the two measurements describe
//                                                          the same movement. The wheel odometry tend to diverge from the actual path because of errors
#define ROTATE_REF_SYSTEM_TO_MATCH_WHEEL_ODOMETRY false
#define ROTATION_ANGLE 130

// Minimum distance that the robot needs to travel in order for heading to be computed
// if the minimum distance isn't reached, the previously computed heading will be kept.
// a value of 0.0d basically means that every (non-zero) movement will be considered valid and heading will be computed.
// increasing the value helps to mitigate the spinning effect of the GPS odometry
// at the beginning and at the end of the bag. 
#define DISTANCE_THRESHOLD_TO_COMPUTE_HEADING 0.0d

#define PI 3.14159d
#define DEG_TO_RAD PI/180.0d


// constants used to perform the convertion from LLA to ECEF coordinates
#define A_CONST 6378137.0d
#define B_CONST 6356752.0d
#define E_SQR_CONST 0.006694478197993d // E_SQR = 1 - (B/A)^2

/**
 * gps_to_odom node that reads NavSatFix messages from /fix.
 * For each message received, it publishes an Odometry message representing the ENU location.
 * Requires LLA coordinates for the reference point provided as params in the node's namespace.
*/
class GPSToOdom {
private:
    // LLA coordinates of the reference point
    double lat_r, lon_r, alt_r;

    // ECEF coordinates of the reference point
    double X_r, Y_r, Z_r;

    // the previous heading of the robot, used when the robot stays still
    double lastHeading;

    // node handles, private one used to access the node's params
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate;

    // subscriber for the NatSatFix messages
    ros::Subscriber GPSSub;
    // publisher for the Odometry messages
    ros::Publisher odomPub;

    // The last message sent (position is used to approximate the heading)
    nav_msgs::Odometry lastOdom;

    /**
     * Computes the sin of an angle provided in degrees
     * 
     * @param angle a value in degrees representing an angle
     * @return the sin value of the provided angle 
    */
    static double sinDeg(double angle) {
        // a_rad = a_deg * PI / 180
        return sin(angle * DEG_TO_RAD);
    }

    /**
     * Computes the cos of an angle provided in degrees
     * 
     * @param angle a value in degrees representing an angle
     * @return the cos value of the provided angle 
    */
    static double cosDeg(double angle) {
        // a_rad = a_deg * PI / 180
        return cos(angle * DEG_TO_RAD);
    }

    /**
     * Converts LLA coordinates to ECEF coordinates
     * 
     * @param lat the latitude of the location whose coordinates need to be converted
     * @param lon the longitude of the location whose coordinates need to be converted
     * @param lat the altitude of the location whose coordinates need to be converted
     * @param x the container for the component of the ECEF point that lies on the X axis
     * @param y the container for the component of the ECEF point that lies on the Y axis
     * @param z the container for the component of the ECEF point that lies on the Z axis
    */
    void GPStoECEF(double lat, double lon, double alt,
                    double &x, double &y, double &z) {
        
        // applying the formulae to convert LLA coordinates to ECEF coordinates
        double t = sinDeg(lat);
        double N = A_CONST / sqrt(1.0d - E_SQR_CONST * t * t);

        x = (N + alt) * cosDeg(lat) * cosDeg(lon);
        y = (N + alt) * cosDeg(lat) * sinDeg(lon);
        z = (N * (1.0d - E_SQR_CONST) + alt) * sinDeg(lat);
    }

    /**
     * Converts LLA coordinates to ENU coordinates centered around the reference point
     * 
     * @param lat the latitude of the location whose coordinates need to be converted
     * @param lon the longitude of the location whose coordinates need to be converted
     * @param lat the altitude of the location whose coordinates need to be converted
     * @param e the container for the component of the ENU point that lies on the axis pointing East
     * @param n the container for the component of the ENU point that lies on the axis pointing North
     * @param u the container for the component of the ENU point that lies on the axis pointing Up
    */
    void GPStoENU(double lat, double lon, double alt, 
                    double &e, double &n, double &u) {

        // converting the LLA coordinates to ECEF coordinates
        double X_p, Y_p, Z_p;
        GPStoECEF(lat, lon, alt, X_p, Y_p, Z_p);

        // computing the difference between the reference point from the given point, both in ECEF coordinates
        double  dx = X_p - X_r,
                dy = Y_p - Y_r,
                dz = Z_p - Z_r;

        // applying the formulae to convert ECEF coordinates to ENU coordinates
        e = - sinDeg(lon_r)*dx + cosDeg(lon_r)*dy;
        n = - sinDeg(lat_r)*cosDeg(lon_r)*dx -sinDeg(lat_r)*sinDeg(lon_r)*dy + cosDeg(lat_r)*dz;
        u = + cosDeg(lat_r)*cosDeg(lon_r)*dx +cosDeg(lat_r)*sinDeg(lon_r)*dy + sinDeg(lat_r)*dz;
    }

    /**
     * Computes the distance travelled on a plane given the vertical and horizontal components.
     * It serves as an helper method for computHeading.
     * 
     * @param dx X/East axis component
     * @param dy y/North axis component
    */
    static double dist(double dx, double dy) {
        return sqrt(dx*dx + dy*dy);
    }

    /**
     * Computes an approximation of the robot heading given the most recent movement on the North and East axis.
     * If the robot hasn't moved (enough) on the East-North plane, the last heading will be returned 
     * (which is set to zero at the creation of the node).
     * 
     * @param dEast the component on the East axis of the robot movement
     * @param dNorth the component on the North axis of the robot movement
     * @return an approximation of the robot heading
    */
    double computeHeading(double dEast, double dNorth) {
        // no movement detected => lastHeading is the best approximation for the robot current heading
        if (dist(dEast, dNorth) < DISTANCE_THRESHOLD_TO_COMPUTE_HEADING) {
            return lastHeading;
        } 

        // computing the orientation of the robot movement
        // since the position is only updated 5 times a second
        // that value may diverge a little from the actual one
        return atan2(dNorth,dEast);
    }

public:
    /**
     * Callback function triggered when a GPS message is received.
     * Converts the location from LLA to ENU coordinates and publishes an
     * odometry message representing the robot location according to the GPS
     * 
     * @param msg a pointer to the message containing the GPS data
    */
    void fixCallback(const sensor_msgs::NavSatFixConstPtr& msg) {
        // Copying the message information in the new message
        // (I decided to maintain the "old" message's timestamp because
        // it is linked to the location, which was
        // measured at that precise time)
        nav_msgs::Odometry currentOdom;
        currentOdom.header.seq = msg->header.seq;
        currentOdom.header.stamp = msg->header.stamp;

        // assigning gps_odom as frame id for the message
        std::string frame_id = "gps_odom";
        currentOdom.header.frame_id = frame_id.c_str();

        // converting the LLA coordinates to ENU coordinates
        double e,n,u;
        GPStoENU(msg->latitude, msg->longitude, msg->altitude, e,n,u);

#if ROTATE_REF_SYSTEM_TO_MATCH_WHEEL_ODOMETRY
        // rotating the ENU reference point by ROTATION_ANGLE degrees counter-clockwise around the U axis (if required)
        double x,y;

        x = e * cosDeg(ROTATION_ANGLE) - n * sinDeg(ROTATION_ANGLE);
        y = e * sinDeg(ROTATION_ANGLE) + n * cosDeg(ROTATION_ANGLE);

        e = x;
        n = y;
#endif

        currentOdom.pose.pose.position.x = e;
        currentOdom.pose.pose.position.y = n;
        currentOdom.pose.pose.position.z = u;

        // approximating the robot heading given the two most recent known positions
        double dEast = currentOdom.pose.pose.position.x - lastOdom.pose.pose.position.x,
                dNorth = currentOdom.pose.pose.position.y - lastOdom.pose.pose.position.y;
        lastHeading = computeHeading(dEast, dNorth);

        // building the quaternion representing the robot orientation
        // (pitch is not included in the computation because most of the time that value is uninteresting)
        tf::Quaternion q;
        q.setRPY(0, 0, lastHeading);
        currentOdom.pose.pose.orientation.x = q.getX();
        currentOdom.pose.pose.orientation.y = q.getY();
        currentOdom.pose.pose.orientation.z = q.getZ();
        currentOdom.pose.pose.orientation.w = q.getW();

        // updating the stored position with the last known position (i.e. the current one)
        lastOdom.pose.pose.position.x = currentOdom.pose.pose.position.x;
        lastOdom.pose.pose.position.y = currentOdom.pose.pose.position.y;
        lastOdom.pose.pose.position.z = currentOdom.pose.pose.position.z;

        // publishing the message that was just built
        odomPub.publish(currentOdom);    
    }

    /**
     * Builds the gps_to_odom node object.
     * Subscribes to the /fix topic and advertises the /gps_odom topic.
     * Resets the lastHeading value.
     * Reads the reference point from the node's params.
    */
    GPSToOdom() : nhPrivate("~") {
        // Subscribing to the /fix topic, so that messages from the GPS can be handled
        GPSSub = nh.subscribe("/fix", 1, &GPSToOdom::fixCallback, this);
        // Advertising the /gps_odom topic, used to publish the Odometry of the robot according to the GPS data
        odomPub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 1);

        // resets the current robot heading
        lastHeading = 0.0d;

        // gets the GPS coordinates of the references point
        this->nhPrivate.getParam("lat_r", lat_r);
        this->nhPrivate.getParam("lon_r", lon_r);  
        this->nhPrivate.getParam("alt_r", alt_r);
        
        // computes the ECEF coordinates of the reference point
        GPStoECEF(lat_r, lon_r, alt_r,  X_r, Y_r, Z_r);
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "gps_to_odom");
    GPSToOdom o;
    ros::spin();

    return 0;
}