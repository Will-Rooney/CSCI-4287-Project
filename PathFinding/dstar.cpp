/*
  dstar.cpp

  Author: Will Rooney
  Date Created: 04/05/2017
  Date Modified: 04/25/2017

  Description: Implementation of dstar.h
*/

#include "dstar.h"

DStar::DStar() {

}

void DStar::init() {
  km = 0;
  start = 0;
  initNodes();
  nMap[GOAL_NODE].rhs = 0;
  U.Insert(&nMap[GOAL_NODE], calculateKey(nMap[GOAL_NODE]));
}

Key DStar::calculateKey(Node n) {
  byte g = n.g;
  byte rhs = n.rhs;
  byte f = h(nMap[start].row, nMap[start].col, n.row, n.col) + km;

  byte min = min(g, rhs);
  if (min != BYTE_INF) {
    return Key(min + f, min);
  }
  return Key(BYTE_INF, BYTE_INF);
}

void DStar::updateNode(Node *n) {
  if (n->g != n->rhs && U.contains(n)) {
    U.Update(n, calculateKey(*n));
  }
  else if (n->g != n->rhs && !U.contains(n)) {
    U.Insert(n, calculateKey(*n));
  }
  else if (n->g == n->rhs && U.contains(n)) {
    U.Remove(n);
  }
}

void DStar::addObstacle(byte row, byte col, byte action) {
  byte INF = BYTE_INF, num_neighbors = 4, newCost = 100, c_old;

  // Peek to obstacle node
  if (action == 0 && row > 0)		// Node 'up' from us is an obstacle
    row--;
  else if (action == 1 && col < 9)	// Node 'right' from us is an obstacle
    col++;
  else if (action == 2 && row < 9)	// Node 'down' from us is an obstacle
    row++;
  else if (action == 3 && col > 0)	// Node 'left' from us is an obstacle
    col--;
  /*else {
  	Serial.println("[DStar::addObstacle()] Error: Invalid indexing/action");
  	return;
    }*/

  byte obstacle = INF;
  if ((obstacle = getNodeIndex(row, col)) == INF) {
    //Serial.println("Obstacle detected inbetween nodes");
    // Obstacle is inbetween nodes
    // Only need to update two edges
    // Update (neighbor, obstacle)
    Node *u = &nMap[start];
    Node *v = NULL;
    if (action == 4)
      v = &nMap[4];
    else if (action == 5)
      v = &nMap[1];
    else
      v = u->neighbors[action];
    c_old = u->edgeCost[action];
    u->edgeCost[action] = newCost;
    if (action == 4)
      v->edgeCost[0] = newCost;
    else if (action == 5)
      v->edgeCost[1] = newCost;
    else if (action > 1)
      v->edgeCost[action - 2] = newCost;
    else
      v->edgeCost[action + 2] = newCost;

    // Update (u,v)
    if (((v->g == INF && u->rhs == INF) || u->rhs == c_old + v->g) && *u != nMap[GOAL_NODE]) {
      byte min = INF, j;
      for (j = 0; j < num_neighbors; ++j) {
        Node *neighbor = u->neighbors[j];
        if (neighbor != NULL) {
          byte temp = INF;
          if (neighbor->g != INF)
            temp = u->edgeCost[j] + neighbor->g;
          if (temp < min)
            min = temp;
        }
      }
      u->rhs = min;
    }
    updateNode(u);

    // Update (v,u)
    if (((u->g == INF && v->rhs == INF) || v->rhs == c_old + u->g) && *v != nMap[GOAL_NODE]) {
      byte min = INF, j;
      for (j = 0; j < num_neighbors; ++j) {
        Node *neighbor = v->neighbors[j];
        if (neighbor != NULL) {
          byte temp = INF;
          if (neighbor->g != INF)
            temp = v->edgeCost[j] + neighbor->g;
          if (temp < min)
            min = temp;
        }
      }
      v->rhs = min;
    }
    updateNode(v);
  }
  else {
    //Serial.println("Obstacle detected on a node");
    // Obstacle is on a node
    // Need to update all edges to node
    Node *neighbor = NULL;
    byte steps = 0, i;
    /* Update all nodes affected by the obstacle */

    //nMap[obstacle].g = INF;
    for (i = 0; i < num_neighbors; ++i) {
      neighbor = nMap[obstacle].neighbors[i];
      if (neighbor != NULL) {
        // Update neighbor Cost
        if (i == 0 && neighbor->row == 0 && neighbor->col == 3) {
          c_old = neighbor->edgeCost[1];
          neighbor->edgeCost[1] = newCost;
        }
        else if (i == 1 && neighbor->row == 2 && neighbor->col == 7) {
          c_old = neighbor->edgeCost[0];
          neighbor->edgeCost[0] = newCost;
        }
        else if (i > 1) {
          c_old = neighbor->edgeCost[i - 2];
          neighbor->edgeCost[i - 2] = newCost;
        }
        else {
          c_old = neighbor->edgeCost[i + 2];
          neighbor->edgeCost[i + 2] = newCost;
        }

        // Update (neighbor, obstacle)
        if (((nMap[obstacle].g == INF && neighbor->rhs == INF) || neighbor->rhs == c_old + nMap[obstacle].g) && *neighbor != nMap[GOAL_NODE]) {
          byte min = INF, j;
          for (j = 0; j < num_neighbors; ++j) {
            Node *tempNode = neighbor->neighbors[j];
            if (tempNode != NULL) {
              byte temp = INF;
              if (tempNode->g != INF)
                temp = neighbor->edgeCost[j] + tempNode->g;
              if (temp < min)
                min = temp;
            }
          }
          neighbor->rhs = min;
        }
        updateNode(neighbor);
      }
    }
    // Update Obstacle Cost
    for (i = 0; i < num_neighbors; ++i)
      if (nMap[obstacle].edgeCost[i] != INF)
        nMap[obstacle].edgeCost[i] = newCost;
    nMap[obstacle].rhs = INF;
    updateNode(&nMap[obstacle]);
  }
}

void DStar::computeShortestPath() {
  Node GOAL = nMap[GOAL_NODE];
  byte INF = BYTE_INF;
  byte steps = INF;
  byte i, num_neighbors = 4;
  while (U.TopKey() < calculateKey(nMap[start]) || nMap[start].rhs > nMap[start].g) {
    Node *u = U.Top();
    if (u == NULL) {
      Serial.println("Empty queue!");
      break;
    }
    Key k_old = U.TopKey();
    byte row = u->row, col = u->col;
    byte u_index = getNodeIndex(row, col);

    Key k_new = calculateKey(*u);
    if (k_old < k_new) {
      U.Update(u, k_new);
    }
    else if (u->g > u->rhs) {
      u->g = u->rhs;
      U.Remove(u);

      // for all s in u->neighbors
      //	if (s != goal) rhs(s) = min(rhs(s), cost(s, u) + g(u))
      //	updateNode(s)
      for (i = 0; i < num_neighbors; ++i) {
        Node *neighbor = u->neighbors[i];
        if (neighbor != NULL) {
          steps = u->edgeCost[i];
          if (*neighbor != GOAL) {
            if (u->g != INF)
              neighbor->rhs = min(neighbor->rhs, steps + u->g);
          }
          updateNode(neighbor);
        }
      }
    }
    else {
      byte g_old = u->g;
      u->g = INF;

      // For all s in u->neighbors Union 'u' *****
      //	rhs(s) = min (for s' in Succ(s) ( cost(s, s') + g(s'))
      for (i = 0; i < num_neighbors; ++i) {
        Node *neighbor = u->neighbors[i];
        if (neighbor != NULL) {
          steps = u->edgeCost[i];
          if ((g_old == INF && neighbor->rhs == g_old) || neighbor->rhs == steps + g_old) {
            if (*neighbor != GOAL) {
              byte min = INF;
              byte j;
              for (j = 0; j < num_neighbors; ++j) {
                Node *tempNeighbor = neighbor->neighbors[j];
                if (tempNeighbor != NULL) {
                  byte steps_temp = neighbor->edgeCost[j];
                  byte rhs_temp = INF;
                  if (tempNeighbor->g != INF)
                    rhs_temp = steps_temp + tempNeighbor->g;
                  if (rhs_temp < min)
                    min = rhs_temp;
                }
              }
              neighbor->rhs = min;
            }
          }
          updateNode(neighbor);
        }
      }

      // Now again for 'u'. cost(u, u) == 0
      if (u->rhs == g_old) {
        if (*u != GOAL) {
          byte min = INF;
          for (i = 0; i < num_neighbors; ++i) {
            Node *neighbor = u->neighbors[i];
            if (neighbor != NULL) {
              byte steps_temp = u->edgeCost[i];
              byte rhs_temp = INF;
              if (neighbor->g != INF)
                rhs_temp = steps_temp + neighbor->g;
              if (rhs_temp < min)
                min = rhs_temp;
            }
          }
          u->rhs = min;
        }
      }
      updateNode(u);
    }
  }
}

void DStar::recomputeShortestPath(byte rowStart, byte colStart, byte &action, byte &actionCount) {
  // Recalculate shortest path
  byte old_action = action;
  backtrack(rowStart, colStart, action, actionCount);
  Serial.print("Returning to and Recomputing from (");
  Serial.print(nMap[start].row);
  Serial.print(", ");
  Serial.print(nMap[start].col);
  Serial.print(")\n\n");
  km = km + h(nMap[last].row, nMap[last].col, nMap[start].row, nMap[start].col);
  last = start;
  addObstacle(rowStart, colStart, old_action);
  computeShortestPath();
}

byte DStar::h(byte row1, byte col1, byte row2, byte col2) {
  /* Calculate Euclidiean distance between two points */
  byte x = col2 - col1;
  byte y = row2 - row1;
  byte dist = x * x + y * y;
  dist = sqrt(dist);
  return dist;
}

byte DStar::getNodeIndex(byte row, byte col) {
  byte index = BYTE_INF;
  switch (row) {
    case 0:
      switch (col) {
        case 0:
          index = 0; // Node 1
          break;
        case 3:
          index = 1; // Node 2
          break;
      }
      break;
    case 2:
      switch (col) {
        case 2:
          index = 2; // Node 3
          break;
        case 3:
          index = 3; // Node 4
          break;
        case 7:
          index = 4; // Node 5
          break;
      }
      break;
    case 3:
      if (col == 9)
        index = 5; // Node 6
      break;
    case 4:
      switch (col) {
        case 0:
          index = 6; // Node 7
          break;
        case 3:
          index = 7; // Node 8
          break;
        case 7:
          index = 8; // Node 9
          break;
      }
      break;
    case 5:
      switch (col) {
        case 7:
          index = 9; // Node 10
          break;
        case 9:
          index = 10; // Node 11
          break;
      }
      break;
    case 6:
      switch (col) {
        case 0:
          index = 11; // Node 12
          break;
        case 1:
          index = 12; // Node 13
          break;
        case 5:
          index = 13; // Node 14
          break;
        case 7:
          index = 14; // Node 15
          break;
      }
      break;
    case 8:
      switch (col) {
        case 3:
          index = 15; // Node 16
          break;
        case 5:
          index = 16; // Node 17
          break;
      }
      break;
    case 9:
      if (col == 9)
        index = 17; // Node 18
      break;
  }
  return index;
}

void DStar::getNextAction(byte& action, byte& actionCount) { // Find the cheapest successor of start : for all neighbors n -> min( cost(this, n) + g(n))
  byte row = nMap[start].row;
  byte col = nMap[start].col;
  byte inf = BYTE_INF;
  byte min = inf;
  byte cost = inf;
  action = inf;
  actionCount = inf;

  byte i, size = 4, minAction = inf, minCount = inf;
  for (i = 0; i < size; ++i) {
    Node *neighbor = nMap[start].neighbors[i];
    cost = nMap[start].edgeCost[i];
    /*Serial.print("Action: ");
      Serial.print(i);
      Serial.print(" | Edge cost: ");
      Serial.print(steps);
      Serial.println();*/
    if (neighbor != NULL) {
      byte temp = inf;
      /*Serial.print("G = ");
        Serial.print(nMap[neighbor].g);
        Serial.print(" | rhs = ");
        Serial.print(nMap[neighbor].rhs);
        Serial.println();*/
      if (neighbor->g != inf)
        temp = cost + neighbor->g;
      if (temp <= min) {
        min = temp;
        minAction = i;
        minCount = cost;
      }
    }
  }
  action = minAction;
  actionCount = minCount;
  if (row == 0 && col == 3 && minAction == 1) {
    // moving on a curve from node #2 to node #5
    action = 4;
    start = 4;
  }
  else if (row == 2 && col == 7 && minAction == 0) {
    action = 5;
    start = 1;
  }
  else if (action != -1 && actionCount != inf) {
    switch (action) {
      case 0: // up
        row = row - actionCount;
        start = getNodeIndex(row, col);
        break;
      case 1: // right
        col = col + actionCount;
        start = getNodeIndex(row, col);
        break;
      case 2: // down
        row = row + actionCount;
        start = getNodeIndex(row, col);
        break;
      case 3: // left
        col = col - actionCount;
        start = getNodeIndex(row, col);
        break;
    }
  }
}

void DStar::getActions(bool *availActs) {
  if (nMap[start].neighbors[0])
    availActs[0] = true;
  else
    availActs[0] = false;

  if (nMap[start].neighbors[1])
    availActs[1] = true;
  else
    availActs[1] = false;

  if (nMap[start].neighbors[2])
    availActs[2] = true;
  else
    availActs[2] = false;

  if (nMap[start].neighbors[3])
    availActs[3] = true;
  else
    availActs[3] = false;
}
void DStar::backtrack(byte row, byte col, byte& action, byte& actionCount) {
  byte offset = 0, node = BYTE_INF;

  // reverse the action
  if (action == 4) {
    // Path was blocked going to node #5 from node #2
    action = 6;
    start = 1;
    return;
  }
  else if (action == 5) {
    // Path was blocked going to node #2 from node #5
    action = 7;
    start = 4;
    return;
  }
  else if (action > 1)
    action = action - 2;
  else
    action = action + 2;

  if ((node = getNodeIndex(row, col)) == BYTE_INF) {
    offset++;
    while ((node = getNodeIndex(row, col)) == BYTE_INF) {
      offset++;
      if (action == 0 && row > 0)		// Node 'up'
        row--;
      else if (action == 1 && col < 9)	// Node 'right'
        col++;
      else if (action == 2 && row < 9)	// Node 'down'
        row++;
      else if (action == 3 && col > 0)	// Node 'left' from us is an obstacle
        col--;
      else {
        Serial.println("[DStar::backtrack()] Error: Invalid indexing/action");
        return;
      }
    }
  }
  actionCount = offset;
  start = node;
}



/* NODE COORDINATES & NEIGHBOR STEP COUNT SETTINGS */
void DStar::initNodes() {
  byte INF = 255;

  // Node #1 - START
  nMap[0].setPos(0, 0);
  nMap[0].setEdgeCosts(INF, 3, 4, INF); // UP, RIGHT, DOWN, LEFT
  nMap[0].setNeighbors(NULL, &nMap[1], &nMap[6], NULL);

  // Node #2
  nMap[1].setPos(0, 3);
  nMap[1].setEdgeCosts(INF, 8, 2, 3);
  nMap[1].setNeighbors(NULL, &nMap[4], &nMap[3], &nMap[0]);

  // Node #3
  nMap[2].setPos(2, 2);
  nMap[2].setEdgeCosts(INF, 1, INF, INF);
  nMap[2].setNeighbors(NULL, &nMap[3], NULL, NULL);

  // Node #4
  nMap[3].setPos(2, 3);
  nMap[3].setEdgeCosts(2, 4, 2, 1);
  nMap[3].setNeighbors(&nMap[1], &nMap[4], &nMap[7], &nMap[2]);

  // Node #5
  nMap[4].setPos(2, 7);
  nMap[4].setEdgeCosts(8, INF, 2, 4);
  nMap[4].setNeighbors(&nMap[1], NULL, &nMap[8], &nMap[3]);

  // Node #6
  nMap[5].setPos(3, 9);
  nMap[5].setEdgeCosts(INF, INF, 2, INF);
  nMap[5].setNeighbors(NULL, NULL, &nMap[10], NULL);

  // Node #7
  nMap[6].setPos(4, 0);
  nMap[6].setEdgeCosts(4, 3, 2, INF);
  nMap[6].setNeighbors(&nMap[0], &nMap[7], &nMap[11], NULL);

  // Node #8
  nMap[7].setPos(4, 3);
  nMap[7].setEdgeCosts(2, 4, 4, 3);
  nMap[7].setNeighbors(&nMap[3], &nMap[8], &nMap[15], &nMap[6]);

  // Node #9
  nMap[8].setPos(4, 7);
  nMap[8].setEdgeCosts(2, INF, 1, 4);
  nMap[8].setNeighbors(&nMap[4], NULL, &nMap[9], &nMap[7]);

  // Node #10
  nMap[9].setPos(5, 7);
  nMap[9].setEdgeCosts(1, 2, 1, INF);
  nMap[9].setNeighbors(&nMap[8], &nMap[10], &nMap[14], NULL);

  // Node #11
  nMap[10].setPos(5, 9);
  nMap[10].setEdgeCosts(2, INF, 4, 2);
  nMap[10].setNeighbors(&nMap[5], NULL, &nMap[17], &nMap[9]);

  // Node #12
  nMap[11].setPos(6, 0);
  nMap[11].setEdgeCosts(2, 1, INF, INF);
  nMap[11].setNeighbors(&nMap[6], &nMap[12], NULL, NULL);

  // Node #13
  nMap[12].setPos(6, 1);
  nMap[12].setEdgeCosts(INF, INF, INF, 1);
  nMap[12].setNeighbors(NULL, NULL, NULL, &nMap[11]);

  // Node #14
  nMap[13].setPos(6, 5);
  nMap[13].setEdgeCosts(INF, 2, 2, INF);
  nMap[13].setNeighbors(NULL, &nMap[14], &nMap[16], NULL);

  // Node #15
  nMap[14].setPos(6, 7);
  nMap[14].setEdgeCosts(1, INF, INF, 2);
  nMap[14].setNeighbors(&nMap[9], NULL, NULL, &nMap[13]);

  // Node #16
  nMap[15].setPos(8, 3);
  nMap[15].setEdgeCosts(4, 2, INF, INF);
  nMap[15].setNeighbors(&nMap[7], &nMap[16], NULL, NULL);

  // Node #17
  nMap[16].setPos(8, 5);
  nMap[16].setEdgeCosts(2, INF, INF, 2);
  nMap[16].setNeighbors(&nMap[13], NULL, NULL, &nMap[15]);

  // Node #18 - GOAL
  nMap[17].setPos(9, 9);
  nMap[17].setEdgeCosts(4, INF, INF, INF);
  nMap[17].setNeighbors(&nMap[10], NULL, NULL, NULL);
}
