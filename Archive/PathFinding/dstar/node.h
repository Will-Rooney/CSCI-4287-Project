/*
* node.h
*
* Author: Will Rooney
* Date Created: 03/24/2017
* Date Modified: 04/16/2017
*
* Description:
*  Defines class Node with  (x,y) coordinates,
*  came from node (parent), neighboring nodes,
*  and traversal costs
*/

#ifndef NODE_H_DSTAR
#define NODE_H_DSTAR

#include <cstddef>
#include <limits>
#include <math.h>
#include <vector>

#define NODE_MAX_NEIGHBORS 	4	// Maximum number of paths that can be taken from a particular node 4
#define NUM_NODES			23	// Graph size = 22 Nodes + Goal Node
#define GOAL_NODE			0	// Goal Node nMap Index 

class Node {
public:
	Node();
	Node(float x, float y, Node *n1, Node *n2, Node *n3, Node *n4);

	bool addNeighbor(Node *n);
	bool deleteNeighbor(Node *n);
	void setNeighborNodes(Node *n1, Node *n2, Node *n3, Node *n4);
	void calcNeighborCosts();

	Node getNext(); // Get minium neighbor traversal cost

	void setPos(float x, float y) { this->x = x; this->y = y; }

	float dist(Node n);

	void swap(Node& n) {
		std::swap(this->x, n.x);
		std::swap(this->y, n.y);
		std::swap(this->rhs, n.rhs);
		std::swap(this->g, n.g);
		std::swap(this->neighbors, n.neighbors);
	}
	Node& operator=(Node n) {
		n.swap(*this);
		return *this;
	}

	/*void printNode() {
		std::cout << "Node properties:\n";
		std::cout << "\t(x, y) = " << '(' << x << ", " << y << ")\n";
		std::cout << "\tNeighbor 1 -> Address: " << neighbors[0].first << " | Cost: " << neighbors[0].second << '\n';
		std::cout << "\tNeighbor 2 -> Address: " << neighbors[1].first << " | Cost: " << neighbors[1].second << '\n';
		std::cout << "\tNeighbor 3 -> Address: " << neighbors[2].first << " | Cost: " << neighbors[2].second << '\n';
		std::cout << "\tNeighbor 4 -> Address: " << neighbors[3].first << " | Cost: " << neighbors[3].second << '\n';
	}*/

	bool operator == (const Node& rhs) const { 
		return (this->x == rhs.x && this->y == rhs.y);
	}
	bool operator != (const Node& rhs) const { return !(*this == rhs); }

	// Each neighbor has an edge cost associated with it ( e.g., c(this, neighbor) )
	std::pair<Node*, float> neighbors[NODE_MAX_NEIGHBORS];
	float x, y, g, rhs;
};

#endif