#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>

#include <queue>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoalPublisher {
private:
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate;

    std::queue<move_base_msgs::MoveBaseGoal> goals;

    void loadWayPoints(std::string inputFile){
        std::ifstream file(inputFile);
        std::string line;
    
        if (!file.is_open()) {
            ROS_INFO("FILE NOT FOUND");
            return;
        }

        while (std::getline(file, line)) {
            std::vector<std::string> wpData;
            std::stringstream s(line);
            std::string e;
            while (std::getline(s, e, ',')){
                wpData.push_back(e);
            }
            
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.pose.position.x = std::stod(wpData.at(0));    
            goal.target_pose.pose.position.y = std::stod(wpData.at(1));    
            geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(std::stod(wpData.at(2)));
            goal.target_pose.pose.orientation = q;

            this->goals.push(goal);
        }

        file.close();
        return;
    }

    void publishNextGoal() {
        if(this->goals.empty()) {
            ROS_INFO("GOAL QUEUE IS EMPTY");
            return;
        }

        MoveBaseClient client("move_base", true);
        while(!client.waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for the action server");
        }

        move_base_msgs::MoveBaseGoal nextGoal = this->goals.front();
        this->goals.pop();

        ROS_INFO("NEW GOAL GOAL: X: %f , Y: %f",nextGoal.target_pose.pose.position.x, nextGoal.target_pose.pose.position.y);

        nextGoal.target_pose.header.frame_id = "map";
        nextGoal.target_pose.header.stamp = ros::Time::now();

        client.sendGoal(nextGoal);

        client.waitForResult();

        if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("GOAL REACHED");
        }
        else {
            ROS_INFO("FAILED TO REACH GOAL");
        }
    }

public:
    GoalPublisher() : nhPrivate("~") {}

    void run() {
        std::queue<move_base_msgs::MoveBaseGoal> emptyQueue;
        std::swap(goals, emptyQueue);

        std::string inputFile;
        nhPrivate.getParam("input_file", inputFile);

        this->loadWayPoints(inputFile);

        while(!this->goals.empty()) {
            this->publishNextGoal();
        }
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "goal_publisher");
    GoalPublisher o;

    o.run();

    return 0;
}