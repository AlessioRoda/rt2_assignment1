#include <chrono>
#include <functional>
#include <memory>
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

namespace rt2_assignment1
{

    class StateMachine : public rclcpp::Node
    {

        public: 

            StateMachine(const rclcpp::NodeOptions & options)
            : Node("state_machine_component")
            { 
            //initialize the publisher, the subscriber, client1, client2
            service=this->create_service<rt2_assignment1::srv::Command>("/user_interface", std::bind(&StateMachine::user_interface, this, _1));
            client_p = this->create_client<rt2_assignment1::srv::Position>("/go_to_point");
            clinet_random= this->create_client<rt2_assignment1::srv::RandomPosition("/position_server");


            this->position_server_clbk();

            }


        void position_server_clbk(){
            auto req=std::make_shared<rt2_assignment1::srv::RandomPosition::Request>();
            req->x_max=5.0;
            req->x_min=-5.0;
            req->y_max=5.0;
            req->y_min=-5.0;

            using ServiceResponseFuture = rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedFuture;
            auto received_callback=[this](ServiceResponseFuture future){
            
            if(start)
            {
                auto req= std::make_shared<rt2_assignment1::srv::Position::Request>();
                req->x=future.get()->x;
                req->y=future.get()->y;
                req->theta=future.get()->theta;
                auto send_result= client_p->async_send_request(req);
            }
            
            };
            auto send_result= client_random->async_send_request(req, received_callback);
        }


        private:

            bool start=false;

            rclcpp::Client<rt2_assignment1::srv::Position>::SharedPtr client_p;
            rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedPtr clinet_random;
            rclcpp::Service<rt2_assignment1::srv::Command>::SharedPtr service;



            void user_interface(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<rt2_assignment1::srv::Command::Request> req,
            const std::shared_ptr<rt2_assignment1::srv::Command::Response> res)
             {
                (void)request_header;
                
                    if (req.command == "start"){
                        start = true;
                    }
                    else {
                        start = false;
                    }

                }
                
            };
    
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::StateMachine)

