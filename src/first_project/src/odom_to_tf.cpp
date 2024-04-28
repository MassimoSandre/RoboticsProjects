#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf/transform_broadcaster.h>

/**
 * odom_to_tf node that reads Odometry messages from /input_odom (can be remapped).
 * For each message received, it gets broadcasted as a tf transformation.
 * Requires root and child frame provided as params in the node's namespace.
*/
class OdomToTF {
private:
    // node handles, private one used to access the node's param
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate;

    // subscriber for the Odometry messages
    ros::Subscriber odomSub;
    
    // broadcaster for the transformation
    tf::TransformBroadcaster broadcaster;

    // the root and child frame
    std::string rootFrame;
    std::string childFrame;

public:
    /**
     * Callback function triggered when an Odometry message is received.
     * Broadcasts the data through TF.
     * 
     * @param msg a pointer to the message containing the Odometry data
    */
    void odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
        tf::Transform transform;

        // setting the location of the robot
        transform.setOrigin(tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z));
        
        // setting the heading of the robot
        tf::Quaternion q;
        quaternionMsgToTF(msg->pose.pose.orientation, q);
        transform.setRotation(q);

        // broadcasting the transformation
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),rootFrame.c_str(),childFrame.c_str()));
    }

    /**
     * Builds the odom_to_tf node object.
     * Subscribes to the /input_odom topic.
     * Reads the root and child frame from the node's params.
    */
    OdomToTF() : nhPrivate("~") {
        odomSub = nh.subscribe("/input_odom", 1, &OdomToTF::odometryCallback, this);
        
        nhPrivate.getParam("root_frame", rootFrame);
        nhPrivate.getParam("child_frame", childFrame);
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "odom_to_tf");
    OdomToTF o;
    ros::spin();

    return 0;
}