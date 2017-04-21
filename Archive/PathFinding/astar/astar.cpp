/*
* astar.h
*
* Author: Will Rooney
* Date Created: 03/24/2017
* Date Modified: 04/05/2017
*
* Description: A* Search algorithm modfied to support multiple searches and keeps track of obstructions detected for search path
*/

#include "astar.h"

AStar::AStar() {
	nMap.resize(NUM_NODES);
	initStaticNodes();
}

/* DEBUGGIN/TESTING FUNCTION */
std::string AStar::getNodeName(Node n) {
	int i;
	for (i = 0; i < NUM_NODES; ++i) {
		if (nMap[i] == n) {
			if (i == GOAL_NODE)
				return "Goal";
			else
				return std::to_string(i);
		}
	}
	return "Start";
}

std::stack<Node> AStar::computeShortestPath(Node start) {
	std::stack<Node> path;
	// OPEN set := start; start G score := 0; start H score heuristic(start, goal)
	start.g = 0;
	start.f = dist(start, nMap[GOAL_NODE]);
	NodePQueue OPEN; // Priority queue of Node pointers
	OPEN.push(&start);

	while (!OPEN.empty()) {
		// current := node in open set having lowest fScore value
		Node *current = OPEN.pop();

		// if (current = goal) return path
		if (*current == nMap[GOAL_NODE]) {
			// return reconstruct_path(current)
			while (*current != start) {
				path.push(*current);
				current = current->parent;
			}
			reset(); // reset Closed nodes and scores for next search
			return path;
		}

		// remove current from openset
		// removed in call to OPEN.pop()
	
		// add current to closed set
		current->Closed = true;

		// Expand current's neighbors
		Node *neighbor;
		for (int i = 0; i < NODE_MAX_NEIGHBORS; ++i) {
			neighbor = current->neighbors[i];
			if (neighbor != NULL && !neighbor->Closed) { // if (neighbor in closed set) ignore neighbor already evaluated
				float temp_g = current->g + dist(*current, *neighbor);
				if (!OPEN.contains(neighbor)) { // Discover a new node
					OPEN.push(neighbor);
				}
				else if (temp_g >= neighbor->g) {
					continue; // not a better path
				}
				// best path until now, record it
				neighbor->parent = current;
				neighbor->g = temp_g;
				neighbor->f = temp_g + dist(*neighbor, nMap[GOAL_NODE]);
			}
		}
	}
	// return failure
	std::cerr << "[AStar::computeShortestPath] Error: Search Failed\n";
	return path;
}

void AStar::removeEdge(Node &n1, Node &n2) {
	// Find the nMap indexes for n1/n2
	int i = 0, j = 0, size = NUM_NODES;
	int n1_i = -1, n2_i = -1;
	for (; i < NUM_NODES; ++i) {
		if (n1 == nMap[i]) {
			n1_i = i;
		}
		if (n2 == nMap[i]) {
			n2_i = i;
		}
	}

	// Allows at least one edge to be removed:
	// in the situation where the initial start node is
	// blocked in a particular direction, the start node
	// is not part of the map so no one in the map 'knows' 
	// the initial start node

	if (n1_i == -1) {
		std::cerr << "Error: Cannot remove edge. Node 'n1' doesn't exist\n";
	}
	else if (!nMap[n1_i].deleteNeighbor(&nMap[n2_i])) {
		std::cerr << "Failed to delete neighbor for n1\n";
	}
	else {
		// propogate changes to main
		n1.deleteNeighbor(&nMap[n2_i]);
	}

	if (n2_i == -1) {
		std::cerr << "Error: Cannot remove edge. Node 'n2' doesn't exist\n";
	}
	else if (!nMap[n2_i].deleteNeighbor(&nMap[n1_i])) {
		std::cerr << "Failed to delete neighbor for n2\n";
	}
	else {
		// propogate changes to main
		n2.deleteNeighbor(&nMap[n1_i]);
	}
}

float AStar::dist(Node n1, Node n2) {
	/* Calculate Euclidiean distance between two points */
	float x = n2.x - n1.x;
	float y = n2.y - n1.y;
	float dist = pow(x, 2) + pow(y, 2);
	dist = sqrt(dist);
	return dist;
}

Node AStar::createStartNode(int n) {
	if (n < 1 || n >= NUM_NODES) {
		std::cerr << "[AStar::createStartNode] Error: Node does not exist\n";
		return Node();
	}
	return nMap[n];
}

Node AStar::createStartNode(float x, float y, int n1, int n2) {
	return Node(x, y, &nMap[n1], &nMap[n2], NULL, NULL);
}

Node AStar::initGoalNode(int n) {
	if (n < 1 || n >= NUM_NODES) {
		std::cerr << "[AStar::initGoalNode] Error: Node does not exist\n";
		return Node();
	}
	nMap[GOAL_NODE] = nMap[n]; // Node 'n' neighbors do not need to be revised; Goal node only used for terminal checks and distance calculations
	return nMap[GOAL_NODE];
}

Node AStar::initGoalNode(float x, float y, int n1, int n2) {
	if (n1 < 1 || n1 >= NUM_NODES || n2 < 1 || n2 >= NUM_NODES) {
		std::cerr << "[AStar::initGoalNode] Error: Cannot add/remove edges, Node(s) don't exist\n";
		return Node();
	}

	nMap[GOAL_NODE].setPos(x, y);
	nMap[GOAL_NODE].setNeighbors(&nMap[n1], &nMap[n2], NULL, NULL);

	// Remove Edge from n1 to n2
	nMap[n1].deleteNeighbor(&nMap[n2]);
	nMap[n2].deleteNeighbor(&nMap[n1]);

	// Add Edge from n1/n2 to goal
	nMap[n1].addNeighbor(&nMap[GOAL_NODE]);
	nMap[n2].addNeighbor(&nMap[GOAL_NODE]);

	return nMap[GOAL_NODE];
}


void AStar::reset() {
	int i = 0, size = NUM_NODES;
	for (; i < size; ++i) {
		nMap[i].Closed = false;
		nMap[i].g = std::numeric_limits<float>::infinity();
		nMap[i].f = std::numeric_limits<float>::infinity();
		nMap[i].parent = NULL;
	}
}

/* STATIC NODE COORDINATE/NEIGHBOR SETTINGS */
/*
 *	Will need to update when we receive a more accurate map 
 *
 */
void AStar::initStaticNodes() {
	// Node #1
	nMap[1].setPos(2.5, 35.0);
	nMap[1].setNeighbors(&nMap[2], &nMap[7], NULL, NULL);

	// Node #2
	nMap[2].setPos(17.5, 37.5);
	nMap[2].setNeighbors(&nMap[1], &nMap[3], &nMap[4], NULL);

	// Node #3
	nMap[3].setPos(40.0, 40.0);
	nMap[3].setNeighbors(&nMap[2], &nMap[11], NULL, NULL);

	// Node #4
	nMap[4].setPos(12.5, 32.5);
	nMap[4].setNeighbors(&nMap[2], &nMap[5], &nMap[8], NULL);

	// Node #5
	nMap[5].setPos(22.5, 32.5);
	nMap[5].setNeighbors(&nMap[4], &nMap[9], NULL, NULL);

	// Node #6
	nMap[6].setPos(30.0, 37.5);
	nMap[6].setNeighbors(&nMap[10], NULL, NULL, NULL);

	// Node #7
	nMap[7].setPos(2.5, 25.0);
	nMap[7].setNeighbors(&nMap[1], &nMap[8], &nMap[12], NULL);

	// Node #8
	nMap[8].setPos(12.5, 25.0);
	nMap[8].setNeighbors(&nMap[4], &nMap[7], &nMap[9], NULL);

	// Node #9
	nMap[9].setPos(22.5, 25.0);
	nMap[9].setNeighbors(&nMap[5], &nMap[8], &nMap[10], &nMap[14]);

	// Node #10
	nMap[10].setPos(30.0, 25.0);
	nMap[10].setNeighbors(&nMap[6], &nMap[9], &nMap[11], &nMap[15]);

	// Node #11
	nMap[11].setPos(40.0, 25.0);
	nMap[11].setNeighbors(&nMap[3], &nMap[10], &nMap[16], NULL);

	// Node #12
	nMap[12].setPos(3.0, 15.0);
	nMap[12].setNeighbors(&nMap[7], &nMap[13], &nMap[20], NULL);

	// Node #13
	nMap[13].setPos(15.0, 16.0);
	nMap[13].setNeighbors(&nMap[8], &nMap[12], &nMap[14], &nMap[17]);

	// Node #14
	nMap[14].setPos(22.5, 16.5);
	nMap[14].setNeighbors(&nMap[9], &nMap[13], &nMap[15], &nMap[18]);

	// Node #15
	nMap[15].setPos(30.0, 17.0);
	nMap[15].setNeighbors(&nMap[10], &nMap[14], &nMap[16], &nMap[21]);

	// Node #16
	nMap[16].setPos(40.0, 17.5);
	nMap[16].setNeighbors(&nMap[11], &nMap[15], &nMap[22], NULL);

	// Node #17
	nMap[17].setPos(15.0, 10.0);
	nMap[17].setNeighbors(&nMap[13], &nMap[18], NULL, NULL);

	// Node #18
	nMap[18].setPos(22.5, 10.0);
	nMap[18].setNeighbors(&nMap[14], &nMap[17], &nMap[19], NULL);

	// Node #19
	nMap[19].setPos(22.5, 17.0);
	nMap[19].setNeighbors(&nMap[18], NULL, NULL, NULL);

	// Node #20
	nMap[20].setPos(9.0, 2.5);
	nMap[20].setNeighbors(&nMap[12], &nMap[21], NULL, NULL);

	// Node #21
	nMap[21].setPos(30.0, 2.5);
	nMap[21].setNeighbors(&nMap[15], &nMap[20], &nMap[22], NULL);

	// Node #22
	nMap[22].setPos(37.5, 2.5);
	nMap[22].setNeighbors(&nMap[16], &nMap[21], NULL, NULL);
}
