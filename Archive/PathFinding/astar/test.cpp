/*  Author: Will Rooney
*      Date Created: 04/05/2017
*      Last Modification Date: 04/05/2017
*      Program Description: Sandbox to test pathfinding algorithms. Refer to ../map.png to follow the computed path
*/
#include "astar.h"

int main(int argc, char* argv[])
{
	/* Setup Section */
	std::cout << "Initializing...\n";
	AStar search;
	std::stack<Node> path;

	/* Initialize goal (x, y, neighbor1, neighbor2), and get a copy of the goal node
	 * alternatively: initGoalNode(node number) // if goal is placed on existing node
	*/

	Node goal = search.initGoalNode(37.5, 5.0, 16, 22); // create goal inbetween node 16 and 22
	//Node goal = search.initGoalNode(21); // create goal on node 21

	/* Set current position (x, y, neighbor1, neighbor2)
	 * alternatively: createStartNode(node number) // if start is placed on existing node
	*/

	Node current = search.createStartNode(2.5, 30.0, 1, 7); // create start/current inbetween node 1 and 7
	//Node current = search.createStartNode(2); // create start/current on node 2

	// Compute intial shortest path from 'current' node
	std::cout << "Computing initial shortest path...\n\n";
	path = search.computeShortestPath(current);

	if (path.empty()) {
		std::cerr << "Error! No available path.\n";
		return -1;
	}
	Node next = path.top();
	path.pop();

	int increment = 0, obstacle = 5; // when increment == obstacle calculate new path; obstacle is number of steps before path is blocked
	while (1) {
		/* Loop Section */
		increment++;

		std::cout << "Current node:\t" << search.getNodeName(current) << '\n';
		std::cout << "Next node:\t" << search.getNodeName(next) << "\n\n";

		/* Start moving to next node */

		/* Check for obstacles */

		// If path blocked
		if (increment == obstacle) {
			// Example: road blocked from 'current' to 'next'
			increment = 0;

			/* Return to 'current' if necessary */

			// Remove the edge between current and next
			std::cout << "Path blocked! Removing edge between " << search.getNodeName(current) << " and " << search.getNodeName(next) << "\n\n";
			search.removeEdge(current, next);

			// Recalculate shortest path
			path = search.computeShortestPath(current);
			if (path.empty()) {
				std::cerr << "Error! No available path.\n";
				std::cin.get();
				return -1;
			}
			std::cout << "New path calculated\n\n";

			// Reset next node from new path
			next = path.top();
			path.pop();

			// Start loop over -> try to navigate to the new 'next' node
			continue;
		}

		// Arrived at next node
		if (next == goal) {
			std::cout << "Reached the goal!\n";
			break;
		}
		std::cout << "Arrived at next node!\n\n";
		current = next;
		next = path.top();
		path.pop();

		std::cin.get();
	}
	std::cin.get();
	return 0;
}