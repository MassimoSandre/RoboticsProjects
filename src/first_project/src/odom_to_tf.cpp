#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf/transform_broadcaster.h>

class odom_to_tf {
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::Subscriber odom_sub;
    
    tf::TransformBroadcaster broadcaster;

    std::string root_frame;
    std::string child_frame;

public:
    void OdomCallback(const nav_msgs::OdometryConstPtr& msg) {
        tf::Transform transform;

        transform.setOrigin(tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z));
        
        tf::Quaternion q;
        quaternionMsgToTF(msg->pose.pose.orientation, q);
        transform.setRotation(q);

        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),root_frame.c_str(),child_frame.c_str()));
    }

    odom_to_tf() : nh_private("~") {
        odom_sub = nh.subscribe("/input_odom", 1, &odom_to_tf::OdomCallback, this);
        
        nh_private.getParam("root_frame", root_frame);
        nh_private.getParam("child_frame", child_frame);
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "odom_to_tf");
    odom_to_tf o;
    ros::spin();

    return 0;
}