
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_server/SimplePickAction.h>
#include <arm_server/SimplePlaceAction.h>

typedef actionlib::SimpleActionClient<arm_server::SimplePickAction> pick_client_t;
typedef actionlib::SimpleActionClient<arm_server::SimplePlaceAction> place_client_t;

/******************************** PICK CALLBACKS *******************************************/
// Called once when the goal completes
void pickDoneCb(const actionlib::SimpleClientGoalState& state,
            const arm_server::SimplePickResultConstPtr& result)
{
    ROS_INFO("[pick_client]: finished in state [%s]", state.toString().c_str());

    ROS_INFO("[pick_client]: answer - x: %f, y: %f, z: %f", result->x, result->y, result->z);
    ros::shutdown();
}

// Called once when the goal becomes active
void pickActiveCb()
{
    ROS_INFO("[pick_client]: goal just went active");
}

// Called every time feedback is received for the goal
void pickFeedbackCb(const arm_server::SimplePickFeedbackConstPtr& feedback)
{
    ROS_INFO("[pick_client]: feedback - x: %f, y: %f, z: %f, distance: %f",
             feedback->x,
             feedback->y,
             feedback->z,
             feedback->distance);
}

/******************************************************************************************/

/******************************** PLACE CALLBACKS *******************************************/
// Called once when the goal completes
void placeDoneCb(const actionlib::SimpleClientGoalState& state,
                const arm_server::SimplePlaceResultConstPtr& result)
{
    ROS_INFO("[place_client]: finished in state [%s]", state.toString().c_str());

    ROS_INFO("[place_client]: answer - x: %f, y: %f, z: %f", result->x, result->y, result->z);
    ros::shutdown();
}

// Called once when the goal becomes active
void placeActiveCb()
{
    ROS_INFO("[place_client]: goal just went active");
}

// Called every time feedback is received for the goal
void placeFeedbackCb(const arm_server::SimplePlaceFeedbackConstPtr& feedback)
{
    ROS_INFO("[place_client]: feedback - x: %f, y: %f, z: %f, distance: %f",
             feedback->x,
             feedback->y,
             feedback->z,
             feedback->distance);
}

/******************************************************************************************/


void pickDemo()
{
    pick_client_t pick_client("simple_pick", true);

    // wait for server infinite time
    ROS_INFO("[pick_client]: waiting for pick_server...");

    pick_client.waitForServer();

    ROS_INFO("[pick_client]: ready");

    // build goal
    arm_server::SimplePickGoal goal;
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
    pick_client.sendGoal(goal, &pickDoneCb, &pickActiveCb, &pickFeedbackCb);
}

void placeDemo()
{

    place_client_t place_client("simple_place", true);

    // wait for server infinite time
    ROS_INFO("[pick_client]: waiting for place_server...");

    place_client.waitForServer();

    ROS_INFO("[pick_client]: ready");

    // build goal
    arm_server::SimplePlaceGoal goal;
    // set your coordinates frame
    goal.frame_id = "/base_footprint";
    goal.obj_name = "target";
    // set target coordiantes
    goal.x = 0.7;
    goal.y = 0.0;
    goal.z = 0.6;

    // send goal to action server
    place_client.sendGoal(goal, &placeDoneCb, &placeActiveCb, &placeFeedbackCb);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "client_demo_node");
    ros::NodeHandle nh;

    pickDemo();

    //placeDemo();

    ros::spin();

    return 0;
}
