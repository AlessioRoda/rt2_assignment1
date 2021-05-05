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
   //ros::ServiceClient client_p = n.serviceClient<rt2_assignment1::Position>("/go_to_point");
   
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   //rt2_assignment1::Position p;
  // rt2_assignment1::PositionAction p;

   actionlib::SimpleActionClient <rt2_assignment1::PositionAction> p("/go_to_point", true);
   //p.waitForServer();
   rt2_assignment1::PositionGoal goal_position;
   
   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
        ROS_INFO("\nHa letto 'start' ");   
   		client_rp.call(rp);
        

        goal_position.x=rp.response.x;
        goal_position.y=rp.response.y;
        goal_position.theta=rp.response.theta;

        std::cout << "\nGoing to the position: x= " << goal_position.x << " y= " <<goal_position.y << " theta = " <<goal_position.theta << std::endl;
        p.sendGoal(goal_position);

        ROS_INFO("\n Goal inviato");

        p.waitForResult();

        std::cout << "Position reached" << std::endl;
   		
        //p.request.x = rp.response.x;
   		//p.request.y = rp.response.y;
   		//p.request.theta = rp.response.theta;
    }
     else if(start==false)
     {
        p.cancelAllGoals();
     } 

    // std::cout << "\nGoing to the position: x= " << p.request.x << " y= " <<p.request.y << " theta = " <<p.request.theta << std::endl;
  //  std::cout << "\nGoing to the position: x= " << goal_position.x << " y= " <<goal_position.y << " theta = " <<goal_position.theta << std::endl;
    //std::cout << "Position reached" << std::endl;
           
   	}
   return 0;
}
