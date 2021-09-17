/*************************************************************************************************************************//**
 * \file   state_machine.cpp
 * 
 * \brief Main of the architecture: develop a state machine in order to manage the simulation
 * 
 * \version 1.0
 * \author Carmine Recchiuto, Alessio Roda
 * \date   May 2021
 * 
 * description:
 *    The state_machine node is a sort of "main" in the entire architecture: it is connected to all other nodes
 *    (/user_interface, /position_server and /go_to_point). The scope of this node is to get a command from the /user_interface node
 *    via the Command server custom message. If the command is equal to "start", /state_machine node sends a request to the 
 *    /position_server node in order to get a random position in a defined interval. This position will be used to set the parameters for 
 *    the action goal to reach, then the goal is sent to /go_to_point node that will drive the robot to move. Then ther node verifies 
 *    that command is not "start" anymore, in this case it cancels the goal in order to stop the robot.
*****************************************************************************************************************************/


#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include "rt2_assignment1/PositionAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

//Variable to understand if the robot has to move 
bool start = false;

/**
 * bool user_interface(req, res)
 * 
 * \brief Function impemented to get the user's command 
 * 
 * \param req: request done by the client (start/stop the robot)
 * 
 * \param res: respondse generated from the server
 * 
 * \return true if the server finishes to set the "start" variable on the basis of the command received 
 * 
 * description:
 *    This callback checks if the command from the /user_interface node is "start", 
 *    in this case sets to true the correspective variable, otherwise sets it to false
 **/
bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}

/** 
 * int main(argc, argv)
 * 
 * \brief Main function of the node
 * 
 * \param argc: the number of argument passed as parameters
 * 
 * \param argv: the vector of string containing each argument
 * 
 * \return 0 when the program ends
 * 
 * description:
 *    The main function, makes all the initializations as follows:
 * 
 *    client_rp: to ask a random goal at custom service RandomPosition
 *    goal_position: in order to set the random goal generated as a PositionGoal and send it to /go_to_point node
 *    service: read the command sent by the user from the Command service custom message
 * 
 *    After initializations it generates a request for the random position in interval [-5, 5], then executes an infinite loop 
 *    in wich it gets the ranodom position and sends it as a goal position to the /go_to_point node and waits untill
 *    the target is reached. In case in wich the command from /user_interface is "stop" (so start==false) the goal is cancelled
 *    and the robot is stopped, otherwise the loop is executed again.
 *    
 **/
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

        // Check if command has changed since the goal is not reached
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
