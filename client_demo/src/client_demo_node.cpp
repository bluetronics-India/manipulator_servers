
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pick_server/PickAction.h>

typedef actionlib::SimpleActionClient<pick_server::PickAction> client_t;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "client_demo_node");
    ros::NodeHandle nh;


    client_t ac("pick", true);

    // wait for server infinite time
    ROS_INFO("[pick_client]: waiting for pick_server...");

    ac.waitForServer();

    ROS_INFO("[pick_client]: ready");

    pick_server::PickGoal goal;
    ac.sendGoal(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    ros::spin();

    return 0;
}
