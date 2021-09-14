/*************************************************************************************************************************//**
 * \file   position_service.cpp
 * 
 * \brief Generate a random postion in terms of x, y and theta coordinates
 * 
 * \version 1.0
 * \author Alessio Roda
 * \date   September 2021
 * 
 * description: 
 *   This node has the purpose to generate a random postion in terms of x, y and theta coordinates from a range 
 *   that is defined in the /state_machine. Once it has created, it sends it via RandomPosition custom service message, 
 *   so the node works as a server for the generation of the postions for the /state_machine node.
*****************************************************************************************************************************/

#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/**
 * double randMToN(double M, double N)
 * 
 * \brief generate a random number in the defined interval [M, N]
 * 
 * \param M is the minimum number that can be generated
 * 
 * \param N is the maximum number that can be generated
 * 
 * \return a random double number corrensonding to a the position to reach in the interval [M, N]
 * 
 * 
 */
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


/** 
 * bool myrandom (res, req)
 * 
 * \brief function to get the position request and call the function to generate the random position
 * 
 * \param req: request done from an other node with the range of minimum and maximum values that the random position must have
 * 
 * \param res: the reply of the server with the x, y and theta coordinates of the target
 * 
 * \return true: it notifies that it has generated the random position
 * 
 * description:
 *      gets the params for the interval from the request, then call the function randMToN to obtain a number in that interval
 * 
 */
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/** 
 * int main(int argc, char **argv)
 * 
 * \brief The main function of the node 
 * 
 * \param argc: the number of arguent passed as parameters
 * 
 * \param argv: the vector of string containing each argument
 * 
 * \return 0 when the program has finished
 * 
 * description:
 *      Initialize the node and create a new custom service of type RandomPosition on /position_server
 * 
 */
int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
