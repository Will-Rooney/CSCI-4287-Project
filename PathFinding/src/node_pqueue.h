/*
* node_pqueue.h
*
* Author: Will Rooney
* Date Created: 04/05/2017
* Date Modified: 04/21/2017
*
* Description: Priority queue implemented specifically for A* Search algorithm
*	Due to small graph size (~23 nodes) searching rather than sorting is preffered for finding min F value (pop())
*	For more realistic scenarios, where the graph is much larger, a Hash Map and a Heap would be an efficient approach
*/

#ifndef NODE_PQUEUE_H_DSTAR
#define NODE_PQUEUE_H_DSTAR

#include "node.h"

#define NUM_NODES 18

class Key {
public:
	Key() : k1(BYTE_INF), k2(BYTE_INF) {}
	Key(byte k1, byte k2) : k1(k1), k2(k2) {}

	void update(byte k1, byte k2) {
		this->k1 = k1;
		this->k2 = k2;
	}

	Key& operator=(Key k) {
		this->k1 = k.k1;
		this->k2 = k.k2;
		return *this;
	}
	bool operator < (const Key &kp) {
		return (k1 < kp.k1 || (k1 == kp.k1 && k2 < kp.k2));
	}

	byte k1, k2;
};

class NodePQueue {
public:
	NodePQueue();

	Node* Top();
	Key TopKey();

	void Insert(Node *, Key);
	void Update(Node *, Key);
	void Remove(Node *);

	bool contains(Node*);
	bool empty();

private:
	Node *nodeData[NUM_NODES];
	Key keyData[NUM_NODES];
};
#endif
