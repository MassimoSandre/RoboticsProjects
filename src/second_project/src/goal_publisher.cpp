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

/**
 * goal_publisher node that reads waypoints from an input csv file (x,y,theta) and 
 * publishes them as goals for the move base to reach in the map using ActionLib
*/
class GoalPublisher {
private:
    // node handles, private one used to retrieve node's param (input file name)
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate;

    // queue of goals that will be published (a goal is published as soon as the previous one is either reached or aborted)
    std::queue<move_base_msgs::MoveBaseGoal> goals;

    /**
     * Loads the waypoints from a file whose path is given.
     * The waypoints are loaded as move_base_msgs::MoveBaseGoal in the node's 'goals' queue.
     * If an error occurs while loading the waypoints, the method tries to load as many goals
     * as it can (skipping poorly set up lines).
     * It is possible to call this method multiple times: the new waypoints will be added to
     * the queue and they will be published once all the previously loaded waypoints are
     * published (and succeeded/aborted).
     * This node implementation doesn't actually use the multiple calling feature, since
     * this method is called exactly once when the node is run.
     * This node's run method blocks the caller (and it also empties the queue once it is called).
     * Therefore, the only method to open multiple files (or the same file multiple times) is to 
     * wait for the run() method to return, and then proceed with another invocation of the
     * same method (changing the intput_file param in order to change input_file).
     * if the method fails to open the file, the method will return.
     * The run method will find and empty 'goals' queue and thus it will instantly return as well.
     * 
     * @param inputFile string that the path to the file that contains the waypoints
    */
    void loadWayPoints(std::string inputFile){
        // opens the input file
        std::ifstream file(inputFile);
    
        // checking if the file has been successifully opened
        if (!file.is_open()) {
            ROS_INFO("FILE NOT FOUND");
            return;
        }

        std::string line;

        // reading the file line by line
        while (std::getline(file, line)) {
            std::vector<std::string> wpData;
            std::stringstream s(line);
            std::string e;

            // parsing the file's line
            while (std::getline(s, e, ',')){
                wpData.push_back(e);
            }

            // checking if the loaded data
            if(wpData.size() != 3) {
                ROS_INFO("POORLY FORMATTED LINE IN INPUT FILE: " + line)

                // skipping to the next line
                continue;
            }
            
            double x,y,theta;

            // trying to turn the loaded data into doubles
            try {
                x = std::stod(wpData.at(0));    
                y = std::stod(wpData.at(1));
                theta = std::stod(wpData.at(2));
            }
            catch (...) {
                // possible exceptions:
                //      - std::invalid_argument if no convertion could be performed
                //      - std::out_of_range if the converted value would fall out of the range of the result type
                ROS_INFO("POORLY FORMATTED LINE IN INPUT FILE: " + line);

                // skipping to the next line
                continue;;
            }

            move_base_msgs::MoveBaseGoal goal;

            // creating a move_base_msgs::MoveBaseGoal based on the data obtained while parsing the line
            goal.target_pose.pose.position.x = x;
            goal.target_pose.pose.position.y =  y;
            geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(theta);
            goal.target_pose.pose.orientation = q;
        
            // adding the loaded waypoint as a goal in the queue
            this->goals.push(goal);
        }

        // closing the input file
        file.close();
    }

    /**
     * Publishes the next goal in the node's 'goals' queue.
     * The caller (run) is blocked until the goals is reached or aborted
    */
    void publishNextGoal() {
        // checking if there're available goals
        if(this->goals.empty()) {
            ROS_INFO("TRYING TO PUBLISH GOAL BUT QUEUE IS EMPTY");
            return;
        }
        // creating a client
        MoveBaseClient client("move_base", true);
        while(!client.waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for the action server");
        }

        // retrieving the next goal from the queue
        move_base_msgs::MoveBaseGoal nextGoal = this->goals.front();
        this->goals.pop();

        ROS_INFO("NEW GOAL GOAL: X: %f , Y: %f",nextGoal.target_pose.pose.position.x, nextGoal.target_pose.pose.position.y);

        // setting up goal's header
        nextGoal.target_pose.header.frame_id = "map";
        nextGoal.target_pose.header.stamp = ros::Time::now();

        // sending the goal
        client.sendGoal(nextGoal);

        // waiting for the result
        client.waitForResult();

        if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("GOAL REACHED");
        }
        else {
            ROS_INFO("FAILED TO REACH GOAL");
        }
    }

public:
    /**
     * Sets up the private node handle 
    */
    GoalPublisher() : nhPrivate("~") {}

    /**
     * Publishes the goals for the move base to reach.
     * The goals are retrieved from an input file whose path is provided as rosparam.
     * If the file isn't found (or if other errors occur while trying to open the file)
     * this method will return immediately and no goal will be published.
     * If the file is poorly set up, the node still tries to publish all the properly set up
     * waypoints that it can find in the file (it will skip lines that it cannot parse).
    */
    void run() {
        // clearing the goals queue
        std::queue<move_base_msgs::MoveBaseGoal> emptyQueue;
        std::swap(goals, emptyQueue);

        // retrieving the input file path from the rosparam
        std::string inputFile;
        nhPrivate.getParam("input_file", inputFile);

        // loading waypoints fro the file
        this->loadWayPoints(inputFile);

        // publishing goals until there's nothing to publish
        while(!this->goals.empty()) {
            this->publishNextGoal();
        }
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "goal_publisher");
    GoalPublisher o;

    o.run();

    // ros::spin() is missing in order for the node to stop immediatly after all the goals have been published

    return 0;
}