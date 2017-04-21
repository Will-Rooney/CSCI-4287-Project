/*
* node.cpp
*
* Author: Will Rooney
* Date Created: 03/24/2017
* Date Modified: 04/16/2017
*
* Description:
*  Implementation of node.h
*/

#include "node.h"

Node::Node() {
	float inf = std::numeric_limits<float>::infinity();
	this->x = 0.0;
	this->y = 0.0;
	this->rhs = inf;
	this->g = inf;

	/* Set Neighbor Nodes */
	this->neighbors[0].first = NULL;
	this->neighbors[1].first = NULL;
	this->neighbors[2].first = NULL;
	this->neighbors[3].first = NULL;

	/* Set Neighbor Costs */
	this->neighbors[0].second = inf;
	this->neighbors[1].second = inf;
	this->neighbors[2].second = inf;
	this->neighbors[3].second = inf;
	
}

Node::Node(float x, float y, Node *n1, Node *n2, Node *n3, Node *n4) {
	float inf = std::numeric_limits<float>::infinity();
	this->x = x;
	this->y = y;
	this->rhs = inf;
	this->g = inf;

	/* Set Neighbor Nodes */
	this->neighbors[0].first = n1;
	this->neighbors[1].first = n2;
	this->neighbors[2].first = n3;
	this->neighbors[3].first = n4;

	/* Set Neighbor Costs */
	if (n1 != NULL)
		this->neighbors[0].second = dist(*n1);
	else
		this->neighbors[0].second = inf;
	if (n2 != NULL)
		this->neighbors[1].second = dist(*n2);
	else
		this->neighbors[1].second = inf;
	if (n3 != NULL)
		this->neighbors[2].second = dist(*n3);
	else
		this->neighbors[2].second = inf;
	if (n4 != NULL)
		this->neighbors[3].second = dist(*n4);
	else
		this->neighbors[3].second = inf;
}

bool Node::addNeighbor(Node *n) {
	// Check if neighbor already exists
	if (this->neighbors[0].first == n)
		return false;
	if (this->neighbors[1].first == n)
		return false;
	if (this->neighbors[2].first == n)
		return false;
	if (this->neighbors[3].first == n)
		return false;

	// Check for open slot and add new neighbor
	if (this->neighbors[0].first == NULL) {
		this->neighbors[0].first = n;
		this->neighbors[0].second = dist(*n);
		return true;
	}
	else if (this->neighbors[1].first == NULL) {
		this->neighbors[1].first = n;
		this->neighbors[1].second = dist(*n);
		return true;
	}
	else if (this->neighbors[2].first == NULL) {
		this->neighbors[2].first = n;
		this->neighbors[2].second = dist(*n);
		return true;
	}
	else if (this->neighbors[3].first == NULL) {
		this->neighbors[3].first = n;
		this->neighbors[3].second = dist(*n);
		return true;
	}
	// No slots available
	return false;
}

bool Node::deleteNeighbor(Node *n) {
	// Find n and set to NULL
	if (this->neighbors[0].first == n) {
		this->neighbors[0].first = NULL;
		return true;
	}
	else if (this->neighbors[1].first == n) {
		this->neighbors[1].first = NULL;
		return true;
	}
	else if (this->neighbors[2].first == n) {
		this->neighbors[2].first = NULL;
		return true;
	}
	else if (this->neighbors[3].first == n) {
		this->neighbors[3].first = NULL;
		return true;
	}
	// Neighbor does not exist
	return false;
}

void Node::setNeighborNodes(Node *n1, Node *n2, Node *n3, Node *n4) {
	this->neighbors[0].first = n1;
	this->neighbors[1].first = n2;
	this->neighbors[2].first = n3;
	this->neighbors[3].first = n4;
}

void Node::calcNeighborCosts() {
	float inf = std::numeric_limits<float>::infinity();

	/* Get Neighbor Nodes */
	Node *n1 = this->neighbors[0].first;
	Node *n2 = this->neighbors[1].first;
	Node *n3 = this->neighbors[2].first;
	Node *n4 = this->neighbors[3].first;

	/* Set Neighbor Costs */
	if (n1 != NULL)
		this->neighbors[0].second = dist(*n1);
	else
		this->neighbors[0].second = inf;

	if (n2 != NULL)
		this->neighbors[1].second = dist(*n2);
	else
		this->neighbors[1].second = inf;

	if (n3 != NULL)
		this->neighbors[2].second = dist(*n3);
	else
		this->neighbors[2].second = inf;

	if (n4 != NULL)
		this->neighbors[3].second = dist(*n4);
	else
		this->neighbors[3].second = inf;
}

Node Node::getNext() { // Find the cheapest successor of *this : for all neighbors n -> min( cost(this, n) + g(n))
	float min = std::numeric_limits<float>::infinity();
	Node *succ = NULL;
	if (neighbors[0].first != NULL) {
		float temp = neighbors[0].second + neighbors[0].first->g;
		if (temp < min) {
			min = temp;
			succ = neighbors[0].first;
		}
	}
	if (neighbors[1].first != NULL) {
		float temp = neighbors[1].second + neighbors[1].first->g;
		if (temp < min) {
			min = temp;
			succ = neighbors[1].first;
		}
	}
	if (neighbors[2].first != NULL) {
		float temp = neighbors[2].second + neighbors[2].first->g;
		if (temp < min) {
			min = temp;
			succ = neighbors[2].first;
		}
	}
	if (neighbors[3].first != NULL) {
		float temp = neighbors[3].second + neighbors[3].first->g;
		if (temp < min) {
			min = temp;
			succ = neighbors[3].first;
		}
	}
	if (succ == NULL)
		return Node();
	return *succ;
}

float Node::dist(Node n) {
	/* Calculate Euclidiean distance between two points */
	float x = n.x - this->x;
	float y = n.y - this->y;
	float dist = pow(x, 2) + pow(y, 2);
	dist = sqrt(dist);
	return dist;
}

