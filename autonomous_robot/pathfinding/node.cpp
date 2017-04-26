/*
  node.cpp

  Author: Will Rooney
  Date Created: 03/24/2017
  Date Modified: 04/25/2017

  Description:
   Implementation of node.h
*/

#include "node.h"

Node::Node() {
  this->col = 0;
  this->row = 0;
  this->rhs = BYTE_INF;
  this->g = BYTE_INF;

  /* Set Neighbor Costs */
  this->edgeCost[0] = BYTE_INF;
  this->edgeCost[1] = BYTE_INF;
  this->edgeCost[2] = BYTE_INF;
  this->edgeCost[3] = BYTE_INF;

  /* Set Neighbor Nodes */
  this->neighbors[0] = NULL;
  this->neighbors[1] = NULL;
  this->neighbors[2] = NULL;
  this->neighbors[3] = NULL;
}

Node::Node(byte row, byte col, byte up, byte right, byte down, byte left) {
  this->row = row;
  this->col = col;
  this->rhs = BYTE_INF;
  this->g = BYTE_INF;

  /* Set Neighbor Costs */
  this->edgeCost[0] = up;
  this->edgeCost[1] = right;
  this->edgeCost[2] = down;
  this->edgeCost[3] = left;

  /* Set Neighbor Nodes */
  this->neighbors[0] = NULL;
  this->neighbors[1] = NULL;
  this->neighbors[2] = NULL;
  this->neighbors[3] = NULL;
}

void Node::setEdgeCosts(byte up, byte right, byte down, byte left) {
  this->edgeCost[0] = up;
  this->edgeCost[1] = right;
  this->edgeCost[2] = down;
  this->edgeCost[3] = left;
}

void Node::setNeighbors(Node *n1, Node *n2, Node *n3, Node *n4) {
  this->neighbors[0] = n1;
  this->neighbors[1] = n2;
  this->neighbors[2] = n3;
  this->neighbors[3] = n4;
}

void Node::setPos(byte row, byte col) {
  this->row = row;
  this->col = col;
}

Node& Node::operator=(Node n) {
  this->row = n.row;
  this->col = n.col;
  this->g = n.g;
  this->rhs = n.rhs;
  this->edgeCost[0] = n.edgeCost[0];
  this->edgeCost[1] = n.edgeCost[1];
  this->edgeCost[2] = n.edgeCost[2];
  this->edgeCost[3] = n.edgeCost[3];
  this->neighbors[0] = n.neighbors[0];
  this->neighbors[1] = n.neighbors[1];
  this->neighbors[2] = n.neighbors[2];
  this->neighbors[3] = n.neighbors[3];
  return *this;
}

bool Node::operator == (const Node& n) const {
  return (this->row == n.row && this->col == n.col);
}
bool Node::operator != (const Node& n) const {
  return !(*this == n);
}



