/*
* node.h
*
* Author: Will Rooney
* Date Created: 03/24/2017
* Date Modified: 04/05/2017
*
* Description:
*  Defines class Node with  (x,y) coordinates,
*  came from node (parent), neighboring nodes,
*  and traversal costs
*/

#ifndef NODE_H
#define NODE_H

#include <limits>
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
	void setNeighbors(Node *n1, Node *n2, Node *n3, Node *n4);

	void setPos(float x, float y) { this->x = x; this->y = y; }

	void swap(Node& n) {
		std::swap(this->x, n.x);
		std::swap(this->y, n.y);
		std::swap(this->f, n.f);
		std::swap(this->g, n.g);
		std::swap(this->parent, n.parent);
		std::swap(this->neighbors, n.neighbors);
		std::swap(this->Closed, n.Closed);
	}
	Node& operator=(Node n) {
		n.swap(*this);
		return *this;
	}

	bool operator == (const Node& rhs) const { return (this->x == rhs.x && this->y == rhs.y); }
	bool operator != (const Node& rhs) const { return !(*this == rhs); }

	Node *parent;
	Node *neighbors[NODE_MAX_NEIGHBORS];
	float x, y, f, g;
	bool Closed;
};

#endif