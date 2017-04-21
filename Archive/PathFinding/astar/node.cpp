/*
* node.cpp
*
* Author: Will Rooney
* Date Created: 03/24/2017
* Date Modified: 04/05/2017
*
* Description:
*  Implementation of node.h
*/

#include "node.h"

Node::Node() {
	this->x = 0.0;
	this->y = 0.0;
	this->f = std::numeric_limits<float>::infinity();
	this->g = std::numeric_limits<float>::infinity();
	this->parent = NULL;
	this->neighbors[0] = NULL;
	this->neighbors[1] = NULL;
	this->neighbors[2] = NULL;
	this->neighbors[3] = NULL;
	this->Closed = false;
}

Node::Node(float x, float y, Node *n1, Node *n2, Node *n3, Node *n4) {
	this->x = x;
	this->y = y;
	this->f = std::numeric_limits<float>::infinity();
	this->g = std::numeric_limits<float>::infinity();
	this->parent = NULL;
	this->neighbors[0] = n1;
	this->neighbors[1] = n2;
	this->neighbors[2] = n3;
	this->neighbors[3] = n4;
	this->Closed = false;
}

bool Node::addNeighbor(Node *n) {
	// Check if neighbor already exists
	if (this->neighbors[0] == n)
		return false;
	if (this->neighbors[1] == n)
		return false;
	if (this->neighbors[2] == n)
		return false;
	if (this->neighbors[3] == n)
		return false;

	// Check for open slot and add new neighbor
	if (this->neighbors[0] == NULL) {
		this->neighbors[0] = n;
		return true;
	}
	else if (this->neighbors[1] == NULL) {
		this->neighbors[1] = n;
		return true;
	}
	else if (this->neighbors[2] == NULL) {
		this->neighbors[2] = n;
		return true;
	}
	else if (this->neighbors[3] == NULL) {
		this->neighbors[3] = n;
		return true;
	}
	// No slots available
	return false;
}

bool Node::deleteNeighbor(Node *n) {
	// Find n and set to NULL
	if (this->neighbors[0] == n) {
		this->neighbors[0] = NULL;
		return true;
	}
	else if (this->neighbors[1] == n) {
		this->neighbors[1] = NULL;
		return true;
	}
	else if (this->neighbors[2] == n) {
		this->neighbors[2] = NULL;
		return true;
	}
	else if (this->neighbors[3] == n) {
		this->neighbors[3] = NULL;
		return true;
	}
	// Neighbor does not exist
	return false;
}

void Node::setNeighbors(Node *n1, Node *n2, Node *n3, Node *n4) {
	this->neighbors[0] = n1;
	this->neighbors[1] = n2;
	this->neighbors[2] = n3;
	this->neighbors[3] = n4;
}


