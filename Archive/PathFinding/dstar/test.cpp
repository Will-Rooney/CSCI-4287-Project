/*  Author: Will Rooney
*      Date Created: 04/05/2017
*      Last Modification Date: 04/16/2017
*      Program Description: Sandbox to test pathfinding algorithms. Refer to ../map.png to follow the computed path
*/
#include "dstar.h"

int main(int argc, char* argv[])
{
	/* Setup Section */
	std::cout << "Initializing...\n";
	DStar search;
	Node next;

	/* Set current position (x, y, neighbor1, neighbor2)
	* alternatively: createStartNode(node number) // if start is placed on existing node
	*/
	Node current = search.createStartNode(25.0, 300.0, 1, 7); // create start/current inbetween node 1 and 7
	//Node current = search.createStartNode(2); // create start/current on node 2
	

	/* Initialize goal (x, y, neighbor1, neighbor2), and get a copy of the goal node
	 * alternatively: initGoalNode(node number) // if goal is placed on existing node
	*/
	Node goal = search.initGoalNode(375.0, 50.0, 16, 22); // create goal inbetween node 16 and 22
	//Node goal = search.initGoalNode(21); // create goal on node 21


	// Compute intial shortest path
	std::cout << "Computing initial shortest path from (" << current.x << ", " << current.y << ") and (" << goal.x << ", " << goal.y << ")\n\n";
	search.computeShortestPath();


	int step = 0; // when step > var calculate new path; var is number of steps before path is blocked
	while (current != goal) {
		/* Loop Section */
		step++;

		// Next = minimum cost neighbor
		next = current.getNext();
		//std::cout << "Current node:\t" << search.getNodeName(current) << '\n';
		//std::cout << "Next node:\t" << search.getNodeName(next) << "\n\n";

		/* Move to next */

		/* Check for obstacles */

		// If path blocked
		if (step > 4) {
			/* Example: road blocked from 'current' to 'next' */
			//std::cout << "Path blocked! Updating edge cost between " << search.getNodeName(current) << " and " << search.getNodeName(next) << "\n\n";
			step = 0;

			/* Return to 'start' if necessary */

			// Recalculate shortest path; Modifies cost between current & next to infinity and recomputes path
			if (!search.recomputeShortestPath(current, next)) {
				std::cerr << "Failed to recompute path!\n";
				std::cin.get();
				return -1;
			}

			std::cout << "New path calculated\n\n";

			// Start loop over -> try to navigate to the new 'next' node
			continue;
		}

		// Arrived at next node
		std::cout << "Arrived at next node!\n\n";
		current = next;
		std::cin.get();
	}
	std::cout << "Reached the goal!\n";
	std::cin.get();
	return 0;
}