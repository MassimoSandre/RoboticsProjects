#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>

/**
 * lidar_remap node that reads PointCloud2 messages from /os_cloud_node/points.
 * For each message received, it changes the message's frame_id to wheel_odom or gps_odom.
 * The user can switch from one frame_id to the other through rqt_reconfigure.
*/
class lidar_remap {
private:
    // node handle, used to subscribe to /os_cloud_node/points and advertise /pointcloud_remapped
    ros::NodeHandle nh;

    // subscriber for the PointCloud2 messages
    ros::Subscriber points_sub;
    // subscriber for the PointCloud2 messages
    ros::Publisher points_pub;

    // frame_id that the handled messages needs to have
    std::string frame;
    
    // dynamic reconfigure server used to set the frame
    dynamic_reconfigure::Server<first_project::parametersConfig> server;

public:
    /**
     * Callback function triggered when a parameter is updated.
     * It switch the frame to the desired one.
     * 
     * @param config an object containing the param(s)
     * @param level
    */
    void param_callback(first_project::parametersConfig &config, uint32_t level) {
        // the parameter is represented with an enum (=> int_param)
        this->frame = config.int_param == 0 ? "wheel_odom" : "gps_odom";
    }

    /**
     * Callback function triggered when a PointCloud2 message is received.
     * Changes the message's frame_id so that it matches the desired one.
     * The new message is then published through the /pointcloud_remapped topic.
     * 
     * @param msg a pointer to the message containing the PointCloud2 data
    */
    void callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        sensor_msgs::PointCloud2 new_msg = *msg;

        // updating the message frame_id
        new_msg.header.frame_id = this->frame.c_str();

        // setting the message time stamp to now. If this operation is skipped
        // rviz won't display the points because the message is deemed too old.
        new_msg.header.stamp = ros::Time::now();

        points_pub.publish(new_msg);
    }

    /**
     * Builds the lidar_remap node object.
     * Subscribes to the /os_cloud_node/points topic and advertise the /pointcloud_remapped topic.
     * Sets up the dynamic_reconfigure server.
    */
    lidar_remap() {
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