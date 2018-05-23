
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pick_server/PickAction.h>


typedef actionlib::SimpleActionServer<pick_server::PickAction> server_t;

void executeCB(const pick_server::PickGoalConstPtr& goal, server_t* as)
{

    ROS_INFO("[pick_server]: executeCB was called");


    pick_server::PickFeedback feedback;
    pick_server::PickResult result;

    int x = 0;
    while(!as->isPreemptRequested() || ros::ok())
    {
        x++;
        feedback.state = std::to_string(x);

        as->publishFeedback(feedback);
        ros::Duration(1).sleep();

        if (x > 3)
            break;
    }

    result.x = x;

    as->setSucceeded(result);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pick_server_node");
    ros::NodeHandle nh;

    server_t pick_server(nh, "pick", boost::bind(&executeCB, _1, &pick_server), false);
    pick_server.start();
    ROS_INFO("[pick_server]: ready");

    ros::spin();

    return 0;
}