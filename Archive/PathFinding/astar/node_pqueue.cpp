/*
* pqueue.cpp
*
* Author: Will Rooney
* Date Created: 04/05/2017
* Date Modified: 04/05/2017
*
* Description: Priority queue implementation of pqueue.h
*/

#include "node_pqueue.h"

NodePQueue::NodePQueue() {
	this->data.reserve(NUM_NODES);
}

Node* NodePQueue::pop() {
	int i = 0, min_i = -1, size = this->data.size();
	float min = std::numeric_limits<float>::infinity();
	for (; i < size; ++i) {
		if (data[i]->f < min) {
			min = data[i]->f;
			min_i = i;
		}
	}
	if (min_i == -1) {
		std::cerr << "[NodePQueue:pop()] Error: popping an empy queue\n";
		return &Node();
	}
	Node *n = data[min_i];
	data.erase(data.begin() + min_i);
	return n;
}

void NodePQueue::push(Node *n) {
	data.push_back(n);
}

bool NodePQueue::contains(Node* n) {
	int i = 0, size = this->data.size();
	for (; i < size; ++i) {
		if (data[i] == n) return true;
	}
	return false;
}