/*
* pqueue.h
*
* Author: Will Rooney
* Date Created: 04/05/2017
* Date Modified: 04/05/2017
*
* Description: Priority queue implemented specifically for A* Search algorithm
*	Due to small graph size (~23 nodes) searching rather than sorting is preffered for finding min F value (pop())
*/

#ifndef NODE_PQUEUE_H
#define NODE_PQUEUE_H

#include <iostream>
#include <vector>
#include "node.h"

class NodePQueue {
public:
	NodePQueue();

	Node* pop();
	void push(Node *);

	bool contains(Node*);
	bool empty() { return data.empty(); }

private:
	std::vector<Node*> data;
};
#endif