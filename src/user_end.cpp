#include "ros/ros.h"
#include "second_assignment/ChangeVel.h"
#include "std_srvs/Empty.h"

int main(int argc, char **argv)
{
	// initialization of the node
	ros::init(argc, argv, "node");
	ros::NodeHandle nh;
	// declare ServiveClient object
	ros::ServiceClient client = nh.serviceClient<second_assignment::ChangeVel>("/changeVel");
	// declare the object that contains the request and the response of the service used to change the velocity
	second_assignment::ChangeVel srv1;
	// declare the empty service used to reset the position
	std_srvs::Empty reset_srv;

	std::cout << " USER END NODE \n";
	std::cout << "Press 'w' to increase velocity, press 's' to decrease velocity, 'r' to reset robot position, 'q' to quit \n ";
	char ch;

	// while loop used to receive keyboard input by user
	while (ch != 'q')
	{
		// waiting for a keyboard input
		std::cin.clear();
		std::cin >> ch;

		switch (ch)
		{
			// press 'w', a request is sent to the server 'srv1' and the robot will accelerate
		case 'w':
		{
			std::cout << "The robot velocity increased! \n";
			srv1.request.input = 'w';
			client.waitForExistence();
			client.call(srv1);
			std::cout << "Current speed: " << srv1.response.multiplier << "\n " ;
			break;
		}
			// press 's', a request is sent to the server 'srv1' and the robot will decelerate
		case 's':
		{
			std::cout << "The robot velocity decreased!\n";
			srv1.request.input = 'd';
			client.waitForExistence();
			client.call(srv1);
			std::cout << "Current speed: " << srv1.response.multiplier << "\n ";
			break;
		}
			// press 'r', a request is sent to the server 'srv1' and the robot will reset the speed to 1.0,
		
		case 'r':
		{
			std::cout << "RESET Robot Position\n";
			srv1.request.input = 'r';
			client.waitForExistence();
			client.call(srv1);
			ros::service::call("/reset_positions", reset_srv);
			std::cout << "Current speed: " << srv1.response.multiplier << "\n ";
			break;
		}

		// press 'q', to quit the operations
		case 'q':
			break;
		default:
			std::cout << "Wrong command! Retry..\n";
			std::cout << "Press 'w' to increase velocity, press 's' to decrease velocity, 'r' to reset robot position, 'q' to quit \n ";
		}
	}

	std::cout << "QUIT \n";
	return 0;
}
