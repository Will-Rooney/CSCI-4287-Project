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

std::pair<Node*, Key> NodePQueue::Top() {
	int i = 0, min_i = -1, size = this->data.size();
	Key min; // Key inits to [inf, inf]
	for (; i < size; ++i) {
		if (data[i].second < min) {
			min = data[i].second;
			min_i = i;
		}
	}
	if (min_i == -1) {
		// Queue is Empty -> return NULL node with Key priority of infinity
		// Triggers end of D* computeShortestPath()
		Node *nullNode = NULL;
		return std::make_pair(nullNode, Key());
	}
	return data[min_i];
}

void NodePQueue::Insert(Node *n, Key k) {
	data.push_back(std::make_pair(n, k));
}

void NodePQueue::Update(Node *n, Key k) {
	int i = 0, size = this->data.size();
	for (; i < size; ++i) {
		if (data[i].first == n)  {
			data[i].second = k;
			return;
		}
	}
}

void NodePQueue::Remove(Node *n) {
	int i = 0, size = this->data.size();
	for (; i < size; ++i) {
		if (data[i].first == n)  {
			data.erase(data.begin() + i);
			return;
		}
	}
}

bool NodePQueue::contains(Node* n) {
	int i = 0, size = this->data.size();
	for (; i < size; ++i) {
		if (data[i].first == n) return true;
	}
	return false;
}