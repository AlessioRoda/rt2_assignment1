/**
The position_service.cpp provides a server that generates random positions in x, y, theta coordinates. These informations are setted ina a RandomPosition
custom service message, that is shared between the position_service_component to the state_machine_component.
Here the code is developed as a node component, by creating a class RandomServer
**/


#include <inttypes.h>
#include <memory>

#include"rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rt2_assignment1/srv/random_position.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1
{
    class RandomServer : public rclcpp::Node
    {
        public:
            /**Initialize the class constructor  **/
            RandomServer(const rclcpp::NodeOptions & options)
            : Node("position_service_component", options)
            { 
            //Create a service for RandomServer
            position_service=this->create_service<rt2_assignment1::srv::RandomPosition>("/position_server", std::bind(&RandomServer::myrandom, this, _1, _2, _3));

            }


        private:

                 /**Function to set the coordinates for a random position in a defined interval. 
                  * req is the request from the client and has two components x_max, y_max wich are the maximum value in x and y coordinates 
                  * and x_min, y_min wich are the minimum value in x and y coordinates **/
                rclcpp::Service<rt2_assignment1::srv::RandomPosition>::SharedPtr position_service;

                bool myrandom(
                const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<rt2_assignment1::srv::RandomPosition::Request> req,
                const std::shared_ptr<rt2_assignment1::srv::RandomPosition::Response> res)
                    {
                        (void)request_header;

                            /**Set the x, y, theta components with the random number genreated **/
                            res->x = randMToN(req->x_min, req->x_max);
                            res->y = randMToN(req->y_min, req->y_max);
                            res->theta = randMToN(-3.14, 3.14);
                            return true;
                    };


                 /** Function to generate a random number ina defined interval **/
                double randMToN(double M, double N)
                    {     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::RandomServer)

