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

            RandomServer(const rclcpp::NodeOptions & options)
            : Node("position_service_component", options)
            { 
            //initialize the publisher, the subscriber, client1, client2
            command_service=this->create_service<rt2_assignment1::srv::RandomPosition>("/position_server", std::bind(&RandomServer::myrandom, this, _1, _2, _3));

            //this->myrandom();

            }


        private:

                rclcpp::Service<rt2_assignment1::srv::RandomPosition>::SharedPtr command_service;

                bool myrandom(
                const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<rt2_assignment1::srv::RandomPosition::Request> req,
                const std::shared_ptr<rt2_assignment1::srv::RandomPosition::Response> res)
                    {
                        (void)request_header;

                            res->x = randMToN(req->x_min, req->x_max);
                            res->y = randMToN(req->y_min, req->y_max);
                            res->theta = randMToN(-3.14, 3.14);
                            return true;
                    };

                double randMToN(double M, double N)
                    {     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::RandomServer)









/*

double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
*/
