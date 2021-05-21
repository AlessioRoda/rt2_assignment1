/***************************************************************************************************************************//**
 * \file state_machine.cpp
 * 
 * \brief The state_machine node is a sort of the "main" in the entire architecture: it is connected with all the other nodes
 * (/user_interface, /position_server and /go_to_point). The scope of this node is to get the command from the /user_interface node
 * via the Command server custom message. If the command is equal to "start", steate_machine node sends a request to the 
 * /position_server node in order to get a random position in a defined interval. This position will be used to set the parameters for
 * the position to reach via a Position service custom message that is sent to /go_to_point node which will move the robot. 
 * Then there's a loop in which this node verify that command is not "start" anymore, in this case it stops sending new target position
 * to the robot.
 * The code is developped as a ROS2 component in order to load it in a rt2_assignment1 container.
 * 
 * \author Alessio Roda
 * \date   May 2021
************************************************************************************************************************/

#include <chrono>
#include <functional>
#include <memory>
#include <inttypes.h>
#include <string>
#include <cinttypes>
#include <cstdlib>

#include"rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/command.hpp"
#include "rt2_assignment1/srv/random_position.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1
{

    class StateMachine : public rclcpp::Node
    {

        public: 

		/** Constructor of the class StateMachine which has to be considered as a ros2 component. 
         * The scope of the constructor is to initilaize the command_service service in order to get the Command service custom message
         * from the /user_interface component. It also initializes the client_random to ask a new random position to reach and
         * the client_p in order to send the new position to reach to the /go_to_point node
         * 
         * After the initializations there are two while loop to verify that the client connection was succesful
         * 
         *
         **/
        
            StateMachine(const rclcpp::NodeOptions & options)
            : Node("state_machine_component", options)
            { 
            //initialize the publisher, the subscriber, client1, client2
            command_service=this->create_service<rt2_assignment1::srv::Command>("/user_interface", std::bind(&StateMachine::user_interface, this, _1, _2, _3));
           
            client_random= this->create_client<rt2_assignment1::srv::RandomPosition>("/position_server");

            while(!client_random->wait_for_service(std::chrono::seconds(3))){
					if(!rclcpp::ok()){
						RCLCPP_ERROR(this->get_logger(), "random position client interrupted");
						return ;
					}
					RCLCPP_INFO(this->get_logger(), "waiting for random service...");
				}
            
            client_p = this->create_client<rt2_assignment1::srv::Position>("/go_to_point");
		     while(!client_p->wait_for_service(std::chrono::seconds(3))){
					if(!rclcpp::ok()){
						RCLCPP_ERROR(this->get_logger(), "position client interrupted");
						return ;
					}
					RCLCPP_INFO(this->get_logger(), "waiting for position service...");
				}

            }

		/** Public method to make a request to the random_position_component in order to get a random position in a defined 
         * interval. It initializes the parameters for the request, then if the command from user is "start" it sends the request to the 
         * random_position_component node and checks untill the goal has not been reached. In this case it calls recursively itself
         * 
         *
         **/

        void position_server_clbk(){
            auto req=std::make_shared<rt2_assignment1::srv::RandomPosition::Request>();
            req->x_max=5.0;
            req->x_min=-5.0;
            req->y_max=5.0;
            req->y_min=-5.0;

            if(this->start)
            {
                using ServiceResponseFuture = rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedFuture;
                auto received_callback=[this](ServiceResponseFuture future){

                auto req= std::make_shared<rt2_assignment1::srv::Position::Request>();
                req->x=future.get()->x;
                req->y=future.get()->y;
                req->theta=future.get()->theta;

                std::cout << "\nGoing to the position: x = " << req->x << " y = " << req->y << " theta = " << req->theta << std::endl;

                using ServiceResponseFuture = rclcpp::Client<rt2_assignment1::srv::Position>::SharedFuture;
                auto received_pos_callback= [this](ServiceResponseFuture position_future){
                   
                    if(position_future.get()->ok)
                    {
                        printf("\n Posioitn reached!");
                        position_server_clbk();
                    }
                };
                auto send_result= client_p->async_send_request(req, received_pos_callback);
                };
                auto send_result= client_random->async_send_request(req, received_callback);
            }
    
        }


        private:


                bool start=false;

                rclcpp::Client<rt2_assignment1::srv::Position>::SharedPtr client_p;
                rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedPtr client_random;
                rclcpp::Service<rt2_assignment1::srv::Command>::SharedPtr command_service;

                /** Private method callback of the user interface: it has to check if the command custom service message from
                 *  the position_service_component is "true" and in case set to True the correspective vairable
                 *
                **/ 

                bool user_interface(
                const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<rt2_assignment1::srv::Command::Request> req,
                const std::shared_ptr<rt2_assignment1::srv::Command::Response> res)
                {
                    (void)request_header;
                    
                        if (req->command == "start"){
                            this->start = true;
                            position_server_clbk();
                        }
                        else {
                            this->start = false;

                            return true;
                        }

                    }
                
            };
    
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::StateMachine)

