#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include "rt2_assignment1/PositionAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

bool start = false;

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;

   actionlib::SimpleActionClient <rt2_assignment1::PositionAction> p("/go_to_point", true);
   rt2_assignment1::PositionGoal goal_position;

   bool goal_reached=false;

   
   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   		client_rp.call(rp);
        
        goal_position.x=rp.response.x;
        goal_position.y=rp.response.y;
        goal_position.theta=rp.response.theta;

        std::cout << "\nGoing to the position: x= " << goal_position.x << " y= " <<goal_position.y << " theta = " <<goal_position.theta << std::endl;
        p.sendGoal(goal_position);


        while (p.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
           ros::spinOnce();
           if(start==false)
           {
              //Cancel all the goals
              p.cancelAllGoals();
              goal_reached=false;

              break;

           }
            goal_reached=true;
        }
	
    }


    //Notify the goal has been reached only in case robot isn't stopped from user
    if(goal_reached)
    {
         //Notofy we reached the goal
         std::cout << "\nPosition reached" << std::endl;
         goal_reached=false;
    }
           
   	}
   return 0;
}
