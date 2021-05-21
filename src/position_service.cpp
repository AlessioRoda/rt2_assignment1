/*************************************************************************************************************************//**
 * \file   position_service.cpp
 * 
 * \brief This node has the purpose to generate a random postion in terms of x, y and theta coordinates from a range 
 * that is defined in the /state_machine. Once it has created, it sends it via RandomPosition custom service message to 
 * the /state_machine node
 * 
 * \author Alessio Roda
 * \date   May 2021
*****************************************************************************************************************************/

#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/** Generate a random number in a defined interval **/
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


/** Gets the params for the interval from the request, then call the function randMToN to obtain a number in that interval**/
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/** Initialize the node and create a new custom service of type RandomPosition on /position_server **/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
