
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pick_server/PickAction.h>

typedef actionlib::SimpleActionClient<pick_server::PickAction> client_t;

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const pick_server::PickResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());

    ROS_INFO("Answer: %f", result->x);
    ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
    ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const pick_server::PickFeedbackConstPtr& feedback)
{
    ROS_INFO("Got Feedback %s", feedback->state.c_str());
}


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
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);


    ros::spin();

    return 0;
}
