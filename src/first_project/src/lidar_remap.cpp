#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>

class lidar_remap {
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::Subscriber points_sub;
    ros::Publisher points_pub;

    std::string frame;
    
    dynamic_reconfigure::Server<first_project::parametersConfig> server;

public:

    void param_callback(first_project::parametersConfig &config, uint32_t level) {
        this->frame = config.int_param == 0 ? "wheel_odom" : "gps_odom";
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        sensor_msgs::PointCloud2 new_msg = *msg;

        new_msg.header.frame_id = this->frame.c_str();
        new_msg.header.stamp = ros::Time::now();

        points_pub.publish(new_msg);
    }

    lidar_remap() : nh_private("~") {
        points_sub = nh.subscribe("/os_cloud_node/points", 1, &lidar_remap::callback, this);
        points_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1);

        
        dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f;

        f = boost::bind(&lidar_remap::param_callback, this, _1, _2);
        server.setCallback(f);

    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "lidar_remap");
    lidar_remap o;
    ros::spin();

    return 0;
}