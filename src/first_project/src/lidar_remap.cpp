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
class LidarRemap {
private:
    // node handle, used to subscribe to /os_cloud_node/points and advertise /pointcloud_remapped
    ros::NodeHandle nh;

    // subscriber for the PointCloud2 messages
    ros::Subscriber pointsSub;
    // subscriber for the PointCloud2 messages
    ros::Publisher pointsPub;

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
    void paramCallback(first_project::parametersConfig &config, uint32_t level) {
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
    void pointcloud2Callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        sensor_msgs::PointCloud2 newMsg = *msg;

        // updating the message frame_id
        newMsg.header.frame_id = this->frame.c_str();

        // setting the message time stamp to now. If this operation is skipped
        // rviz won't display the points because the message is deemed too old.
        newMsg.header.stamp = ros::Time::now();

        pointsPub.publish(newMsg);
    }

    /**
     * Builds the lidar_remap node object.
     * Subscribes to the /os_cloud_node/points topic and advertise the /pointcloud_remapped topic.
     * Sets up the dynamic_reconfigure server.
    */
    LidarRemap() {
        pointsSub = nh.subscribe("/os_cloud_node/points", 1, &LidarRemap::pointcloud2Callback, this);
        pointsPub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1);

        dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f;

        f = boost::bind(&LidarRemap::paramCallback, this, _1, _2);
        server.setCallback(f);
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "lidar_remap");
    LidarRemap o;
    ros::spin();

    return 0;
}