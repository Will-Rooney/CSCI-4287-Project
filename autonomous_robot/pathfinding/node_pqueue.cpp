/*
  pqueue.cpp

  Author: Will Rooney
  Date Created: 04/05/2017
  Date Modified: 04/21/2017

  Description: Priority queue implementation of pqueue.h
*/

#include "node_pqueue.h"

NodePQueue::NodePQueue() {
  byte i = 0, size = NUM_NODES;
  for (; i < size; ++i)
    nodeData[i] = NULL;
}

Key NodePQueue::TopKey() {
  byte i = 0, size = NUM_NODES;
  Key min; // Key inits to [inf, inf]
  for (; i < size; ++i) {
    if (nodeData[i] != NULL && keyData[i] < min) {
      min = keyData[i];
    }
  }
  return min;
}

Node* NodePQueue::Top() {
  byte i = 0, min_i = BYTE_INF, size = NUM_NODES;
  Key min; // Key inits to [inf, inf]
  for (; i < size; ++i) {
    if (nodeData[i] != NULL && keyData[i] < min) {
      min = keyData[i];
      min_i = i;
    }
  }
  return nodeData[min_i];
}

void NodePQueue::Insert(Node *n, Key k) {
  /*Serial.print("Inserting into queue: (");
    Serial.print(n->row);
    Serial.print(", ");
    Serial.print(n->col);
    Serial.print(")\n");*/
  byte i = 0, size = NUM_NODES;
  for (; i < size; ++i) {
    if (nodeData[i] == NULL) {
      nodeData[i] = n;
      keyData[i] = k;
      return;
    }
  }
  Serial.println("[NodePQueue::Insert()] Error: Queue overflow");
}

void NodePQueue::Update(Node *n, Key k) {
  byte i = 0, size = NUM_NODES;
  for (; i < size; ++i) {
    if (nodeData[i] == n)  {
      keyData[i] = k;
      return;
    }
  }
  Serial.println("[NodePQueue::Update()] Error: Node not in queue");
}

void NodePQueue::Remove(Node *n) {
  byte i = 0, size = NUM_NODES;
  for (; i < size; ++i) {
    if (nodeData[i] == n)  {
      nodeData[i] = NULL;
      return;
    }
  }
  Serial.println("[NodePQueue::Remove()] Error: Node not in queue");
}

bool NodePQueue::contains(Node* n) {
  int i = 0, size = NUM_NODES;
  for (; i < size; ++i) {
    if (nodeData[i] == n) return true;
  }
  return false;
}
