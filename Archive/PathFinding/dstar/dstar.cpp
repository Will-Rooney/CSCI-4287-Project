/*
* dstar.cpp
*
* Author: Will Rooney
* Date Created: 04/05/2017
* Date Modified: 04/16/2017
*
* Description: Implementation of dstar.h
*/

#include "dstar.h"

DStar::DStar() {
	nMap.resize(NUM_NODES);
	km = 0;
	initStaticNodes();
}

/* DEBUGGIN/TESTING FUNCTION */
/*std::string DStar::getNodeName(Node n) {
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
}*/

Key DStar::calculateKey(Node s) {
	return Key(s.g, s.rhs, dist(start, s), km);
}

void DStar::updateNode(Node *n) {
	if (n->g != n->rhs && U.contains(n)) {
		U.Update(n, calculateKey(*n));
	}
	else if (n->g != n->rhs && !U.contains(n)) {
		U.Insert(n, calculateKey(*n));
	}
	else if (n->g == n->rhs && U.contains(n)) {
		U.Remove(n);
	}
}

bool DStar::updateEdgeCost(Node *u, Node *v, float cost) {
	// Similar to removeEdge() in A* search

	if (u == NULL || v == NULL) {
		std::cerr << "Error: Cannot update edge. Node 'u' or 'v' doesn't exist\n";
		return false;
	}
	else {
		// Update U
		float c_old = this->cost(u, v);

		if (u->neighbors[0].first == v)
			u->neighbors[0].second = cost;
		else if (u->neighbors[1].first == v)
			u->neighbors[1].second = cost;
		else if (u->neighbors[2].first == v)
			u->neighbors[2].second = cost;
		else if (u->neighbors[3].first == v)
			u->neighbors[3].second = cost;

		if (u->rhs == c_old + v->g && u != &nMap[GOAL_NODE]) {
			float min = std::numeric_limits<float>::infinity();
			for (int i = 0; i < NODE_MAX_NEIGHBORS; ++i) {
				if (u->neighbors[i].first != NULL) {
					float temp = u->neighbors[i].second + u->neighbors[i].first->g;
					if (temp < min) {
						min = temp;
					}
				}
			}
			u->rhs = min;
		}
		updateNode(u);

		// Update V
		if (v->neighbors[0].first == u)
			v->neighbors[0].second = cost;
		else if (v->neighbors[1].first == u)
			v->neighbors[1].second = cost;
		else if (v->neighbors[2].first == u)
			v->neighbors[2].second = cost;
		else if (v->neighbors[3].first == u)
			v->neighbors[3].second = cost;

		if (v->rhs == c_old + u->g && v != &nMap[GOAL_NODE]) {
			float min = std::numeric_limits<float>::infinity();
			for (int i = 0; i < NODE_MAX_NEIGHBORS; ++i) {
				if (v->neighbors[i].first != NULL) {
					float temp = v->neighbors[i].second + v->neighbors[i].first->g;
					if (temp < min) {
						min = temp;
					}
				}
			}
			v->rhs = min;
		}
		updateNode(v);

		// Check if Goal is blocked
		for (int i = 0; i < NODE_MAX_NEIGHBORS; ++i) {
			if (nMap[GOAL_NODE].neighbors[i].second != std::numeric_limits<float>::infinity())
				return true;
		}
		return false;
	}
}

void DStar::computeShortestPath() {
	while (true) {
		// while (U.TopKey() < calculateKey(start) || start.rhs > start.g)
		// u = U.Top()
		// k old = U.TopKey()
		std::pair<Node*,Key> top = U.Top();
		Node *u = top.first;
		Key k_old = top.second;
		if (u == NULL || !(k_old < calculateKey(start) || start.rhs > start.g))
			break;

		Key k_new = calculateKey(*u);
		if (k_old < k_new) {
			U.Update(u, k_new);
		}
		else if (u->g > u->rhs) {
			u->g = u->rhs;
			U.Remove(u);

			// for all s in u->neighbors
			//	if (s != goal) rhs(s) = min(rhs(s), cost(s, u) + g(u))
			//	updateNode(s)
			int i;
			Node *neighbor;
			for (i = 0; i < NODE_MAX_NEIGHBORS; ++i) {
				neighbor = u->neighbors[i].first;
				if (neighbor == NULL)
					continue;
				else if (*neighbor != nMap[GOAL_NODE]) {
					neighbor->rhs = std::min(neighbor->rhs, cost(neighbor, u) + u->g);
				}
				updateNode(neighbor);
			}

		}
		else {
			float g_old = u->g;
			u->g = std::numeric_limits<float>::infinity();
			
			// For all s in u->neighbors Union 'u' ***** 
			//	rhs(s) = min (for s' in Succ(s) ( cost(s, s') + g(s'))
			int i;
			Node *neighbor;
			for (i = 0; i < NODE_MAX_NEIGHBORS; ++i) {
				neighbor = u->neighbors[i].first;
				if (neighbor == NULL)
					continue;
				if (neighbor->rhs == cost(neighbor, u) + g_old) {
					if (*neighbor != nMap[GOAL_NODE]) {
						float min = std::numeric_limits<float>::infinity();
						int j;
						for (j = 0; j < NODE_MAX_NEIGHBORS; ++j) {
							if (neighbor->neighbors[j].first != NULL) {
								float temp = neighbor->neighbors[j].second + neighbor->neighbors[j].first->g;
								if (temp < min) {
									min = temp;
								}
							}
						}
						neighbor->rhs = min;
					}
				}
				updateNode(neighbor);
			}
			// Now again for 'u'. cost(u, u) == 0
			if (u->rhs == g_old) {
				if (*u != nMap[GOAL_NODE]) {
					float min = std::numeric_limits<float>::infinity();
					for (i = 0; i < NODE_MAX_NEIGHBORS; ++i) {
						if (u->neighbors[i].first != NULL) {
							float temp = u->neighbors[i].second + u->neighbors[i].first->g;
							if (temp < min) {
								min = temp;
							}
						}
					}
					u->rhs = min;
				}
			}
			updateNode(u);
		}
	}
}

bool DStar::recomputeShortestPath(Node &n1, Node n2) {
	// Find the nMap indexes for n1/n2
	int i = 0, size = NUM_NODES;
	Node *current = NULL, *next = NULL;
	for (; i < size; ++i) {
		if (n1 == nMap[i]) {
			current = &nMap[i];
		}
		if (n2 == nMap[i]) {
			next = &nMap[i];
		}
	}

	if (current == NULL)
		current = &start;
	else {
		start = *current;
	}

	if (next == NULL)
		return false;

	// Recalculate shortest path
	km = km + dist(last, start);
	last = start;
	if (!updateEdgeCost(current, next, std::numeric_limits<float>::infinity()))	{	// updateEdgeCost returns true if the goal can still be reached, false otherwise
		std::cout << "Impossible to reach goal!\n";
		return false;
	}
	start = *current;
	computeShortestPath();
	n1 = *current;
	return true;
}

float DStar::cost(Node *u, Node *v) {
	// Find the edge cost from u to v
	// Edge cost = distance(u, v) || infinity
	if (u == NULL || v == NULL)
		return std::numeric_limits<float>::infinity();
	if (u->neighbors[0].first == v)
		return u->neighbors[0].second;
	else if (u->neighbors[1].first == v)
		return u->neighbors[1].second;
	else if (u->neighbors[2].first == v)
		return u->neighbors[2].second;
	else if (u->neighbors[3].first == v)
		return u->neighbors[3].second;
	else
		return std::numeric_limits<float>::infinity();
}

float DStar::dist(Node n1, Node n2) {
	/* Calculate Euclidiean distance between two points */
	float x = n2.x - n1.x;
	float y = n2.y - n1.y;
	float dist = pow(x, 2) + pow(y, 2);
	dist = sqrt(dist);
	return dist;
}

Node DStar::createStartNode(int n) {
	if (n < 1 || n >= NUM_NODES) {
		std::cerr << "[AStar::createStartNode] Error: Node does not exist\n";
		return Node();
	}
	start = nMap[n];
	last = nMap[n];
	return start;
}

Node DStar::createStartNode(float x, float y, int n1, int n2) {
	start = Node(x, y, &nMap[n1], &nMap[n2], NULL, NULL);
	last = start;
	return start;
}

Node DStar::initGoalNode(int n) {
	if (n < 1 || n >= NUM_NODES) {
		std::cerr << "[AStar::initGoalNode] Error: Node does not exist\n";
		return Node();
	}
	nMap[GOAL_NODE] = nMap[n]; // Node 'n' neighbors do not need to be revised; Goal node only used for terminal checks and distance calculations

	nMap[n].rhs = 0;
	U.Insert(&nMap[n], Key(dist(start, nMap[n]), 0));

	return nMap[n];
}

Node DStar::initGoalNode(float x, float y, int n1, int n2) {
	/*
	 * Pre: call to createStartNode() completed successfully. initGoalNode uses start node position to determine its key 
	 */

	if (n1 < 1 || n1 >= NUM_NODES || n2 < 1 || n2 >= NUM_NODES) {
		std::cerr << "[AStar::initGoalNode] Error: Cannot add/remove edges, Node(s) don't exist\n";
		return Node();
	}

	nMap[GOAL_NODE].setPos(x, y);
	nMap[GOAL_NODE].setNeighborNodes(&nMap[n1], &nMap[n2], NULL, NULL);
	nMap[GOAL_NODE].calcNeighborCosts();

	// Remove Edge from n1 to n2
	nMap[n1].deleteNeighbor(&nMap[n2]);
	nMap[n2].deleteNeighbor(&nMap[n1]);

	// Add Edge from n1/n2 to goal
	nMap[n1].addNeighbor(&nMap[GOAL_NODE]);
	nMap[n2].addNeighbor(&nMap[GOAL_NODE]);

	nMap[GOAL_NODE].rhs = 0;
	U.Insert(&nMap[GOAL_NODE], Key(dist(start, nMap[GOAL_NODE]), 0));

	return nMap[GOAL_NODE];
}


/* STATIC NODE COORDINATE/NEIGHBOR SETTINGS */
/*
*	Will need to update when we receive a more accurate map
*
*/
void DStar::initStaticNodes() {
	// Node #1
	nMap[1].setPos(25.0, 350.0);
	nMap[1].setNeighborNodes(&nMap[2], &nMap[7], NULL, NULL);

	// Node #2
	nMap[2].setPos(175.0, 375.0);
	nMap[2].setNeighborNodes(&nMap[1], &nMap[3], &nMap[4], NULL);

	// Node #3
	nMap[3].setPos(400.0, 400.0);
	nMap[3].setNeighborNodes(&nMap[2], &nMap[11], NULL, NULL);

	// Node #4
	nMap[4].setPos(125.0, 325.0);
	nMap[4].setNeighborNodes(&nMap[2], &nMap[5], &nMap[8], NULL);

	// Node #5
	nMap[5].setPos(225.0, 325.0);
	nMap[5].setNeighborNodes(&nMap[4], &nMap[9], NULL, NULL);

	// Node #6
	nMap[6].setPos(300.0, 375.0);
	nMap[6].setNeighborNodes(&nMap[10], NULL, NULL, NULL);

	// Node #7
	nMap[7].setPos(25.0, 250.0);
	nMap[7].setNeighborNodes(&nMap[1], &nMap[8], &nMap[12], NULL);

	// Node #8
	nMap[8].setPos(125.0, 250.0);
	nMap[8].setNeighborNodes(&nMap[4], &nMap[7], &nMap[9], NULL);

	// Node #9
	nMap[9].setPos(225.0, 250.0);
	nMap[9].setNeighborNodes(&nMap[5], &nMap[8], &nMap[10], &nMap[14]);

	// Node #10
	nMap[10].setPos(300.0, 250.0);
	nMap[10].setNeighborNodes(&nMap[6], &nMap[9], &nMap[11], &nMap[15]);

	// Node #11
	nMap[11].setPos(400.0, 250.0);
	nMap[11].setNeighborNodes(&nMap[3], &nMap[10], &nMap[16], NULL);

	// Node #12
	nMap[12].setPos(30.0, 150.0);
	nMap[12].setNeighborNodes(&nMap[7], &nMap[13], &nMap[20], NULL);

	// Node #13
	nMap[13].setPos(150.0, 160.0);
	nMap[13].setNeighborNodes(&nMap[8], &nMap[12], &nMap[14], &nMap[17]);

	// Node #14
	nMap[14].setPos(225.0, 165.0);
	nMap[14].setNeighborNodes(&nMap[9], &nMap[13], &nMap[15], &nMap[18]);

	// Node #15
	nMap[15].setPos(300.0, 170.0);
	nMap[15].setNeighborNodes(&nMap[10], &nMap[14], &nMap[16], &nMap[21]);

	// Node #16
	nMap[16].setPos(400.0, 175.0);
	nMap[16].setNeighborNodes(&nMap[11], &nMap[15], &nMap[22], NULL);

	// Node #17
	nMap[17].setPos(150.0, 100.0);
	nMap[17].setNeighborNodes(&nMap[13], &nMap[18], NULL, NULL);

	// Node #18
	nMap[18].setPos(225.0, 100.0);
	nMap[18].setNeighborNodes(&nMap[14], &nMap[17], &nMap[19], NULL);

	// Node #19
	nMap[19].setPos(225.0, 170.0);
	nMap[19].setNeighborNodes(&nMap[18], NULL, NULL, NULL);

	// Node #20
	nMap[20].setPos(90.0, 25.0);
	nMap[20].setNeighborNodes(&nMap[12], &nMap[21], NULL, NULL);

	// Node #21
	nMap[21].setPos(300.0, 25.0);
	nMap[21].setNeighborNodes(&nMap[15], &nMap[20], &nMap[22], NULL);

	// Node #22
	nMap[22].setPos(375.0, 25.0);
	nMap[22].setNeighborNodes(&nMap[16], &nMap[21], NULL, NULL);

	/* Calculate Edge Costs */
	int i = 1;
	for (; i < NUM_NODES; ++i)
		nMap[i].calcNeighborCosts();
}
