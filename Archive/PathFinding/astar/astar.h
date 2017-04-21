/*
* astar.h
*
* Author: Will Rooney
* Date Created: 03/24/2017
* Date Modified: 04/05/2017
*
* Description: A* Search algorithm modfied to support multiple searches and keeps track of obstructions detected for search path
*/

#ifndef ASTAR_H
#define ASTAR_H

#include <stack>
#include <string>
#include "node_pqueue.h"

class AStar {
public:
	AStar();

	std::stack<Node> computeShortestPath(Node start);

	Node initGoalNode(int n); // Goal is placed on an existing node
	Node initGoalNode(float x, float y, int n1, int n2); // Goal is placed inbetween existing nodes
	Node createStartNode(int n);
	Node createStartNode(float x, float y, int n1, int n2);

	void removeEdge(Node &n1, Node &n2);

	std::string getNodeName(Node n); // For debugging/testing

private:
	void initStaticNodes();
	void reset();

	float dist(Node n1, Node n2);

	std::vector<Node> nMap;
};
#endif