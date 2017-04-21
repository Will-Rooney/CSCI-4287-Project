/*
* node_pqueue.h
*
* Author: Will Rooney
* Date Created: 04/05/2017
* Date Modified: 04/05/2017
*
* Description: Priority queue implemented specifically for A* Search algorithm
*	Due to small graph size (~23 nodes) searching rather than sorting is preffered for finding min F value (pop())
*	For more realistic scenarios, where the graph is much larger, a Hash Map and a Heap would be an efficient approach
*/

#ifndef NODE_PQUEUE_H_DSTAR
#define NODE_PQUEUE_H_DSTAR

#include <iostream>
#include <vector>
#include "node.h"
#include "key.h"

class NodePQueue {
public:
	NodePQueue();

	//Node* Top();
	std::pair<Node*, Key> Top(); // Combing Top and TopKey so search only needs to happen once
	//Key TopKey();

	void Insert(Node *, Key);
	void Update(Node *, Key);
	void Remove(Node *);

	bool contains(Node*);
	bool empty() { return data.empty(); }

private:
	std::vector<std::pair<Node*, Key> > data;
};
#endif