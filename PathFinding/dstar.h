/*
  dstar.h

  Author: Will Rooney
  Date Created: 03/24/2017
  Date Modified: 04/25/2017

  Description: Sven Koenig and Maxim Likhachev's D* Lite Search algorithm.
	The paper can be found here: http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf
*/

#ifndef DSTAR_H
#define DSTAR_H

#include "node_pqueue.h"

#define GOAL_NODE 17

class DStar {
  public:
    DStar();

    void init();

    void computeShortestPath();
    void recomputeShortestPath(byte rowStart, byte colStart, byte& action, byte& actionCount);

    byte h(byte row1, byte col1, byte row2, byte col2);

    byte getNodeIndex(byte row, byte col);
    void getNextAction(byte& action, byte& actionCount);
    void backtrack(byte row, byte col, byte& action, byte& actionCount);


  private:
    void initNodes();

    Key calculateKey(Node n);

    void updateNode(Node *n);
    void addObstacle(byte currentRow, byte currentCol, byte action);

    float km;
    int start, last; // index of start and last nodes
    NodePQueue U;
    Node nMap[NUM_NODES];
};
#endif
