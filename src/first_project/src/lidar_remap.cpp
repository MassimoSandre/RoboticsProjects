#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

class lidar_remap {
private:
    ros::NodeHandle nh;
    ris::NodeHandle nh_private;

    ros::Subscriber points_sub;
    ros::Publisher points_pub;

public:
    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        
    }

    int lidar_remap() : nh_private("~") {
        points_sub = nh.subscribe("/os_cloud_nose/points", 1, &pub_sub::callback, this);
        points_pub = nh.advertise<nav_msgs::Odometry>("/pointcloud_remapped", 1);

    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "lidar_remap");
    lidar_remap o;
    ros::spin();

    return 0;
}