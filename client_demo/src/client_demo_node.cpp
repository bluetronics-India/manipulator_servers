
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pick_server/PickAction.h>

typedef actionlib::SimpleActionClient<pick_server::PickAction> client_t;

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const pick_server::PickResultConstPtr& result)
{
    ROS_INFO("[pick_client]: finished in state [%s]", state.toString().c_str());

    ROS_INFO("[pick_client]: answer - x: %f, y: %f, z: %f", result->x, result->y, result->z);
    ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
    ROS_INFO("[pick_client]: goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const pick_server::PickFeedbackConstPtr& feedback)
{
    ROS_INFO("[pick_client]: feedback - x: %f, y: %f, z: %f, distance: %f",
             feedback->x,
             feedback->y,
             feedback->z,
             feedback->distance);
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

    // build goal
    pick_server::PickGoal goal;
    // set your coordinates frame
    goal.frame_id = "/base_footprint";
    goal.obj_name = "target";
    // set target coordiantes
    goal.x = 0.7;
    goal.y = 0.0;
    goal.z = 0.6;
    // set target cylinder primitives
    goal.h = 0.145;
    goal.w = 0.03;

    // send goal to action server
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ros::spin();

    return 0;
}
