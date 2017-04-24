/*
* dstar.cpp
*
* Author: Will Rooney
* Date Created: 04/05/2017
* Date Modified: 04/21/2017
*
* Description: Implementation of dstar.h
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
	else {
		Serial.println("[DStar::addObstacle()] Error: Invalid indexing/action");
		return;
	}

	byte obstacle = INF;
	if ((obstacle = getNodeIndex(row, col)) == INF) {
		//Serial.println("Obstacle detected inbetween nodes");
		// Obstacle is inbetween nodes
		// Only need to update two edges
		// Update (neighbor, obstacle)
		byte u = start;
		byte v = getActionNeighbor(u, action);
		c_old = nMap[u].actions[action];
		nMap[u].actions[action] = newCost;
		if (action > 1)
			nMap[v].actions[action - 2] = newCost;
		else 
			nMap[v].actions[action + 2] = newCost;

		// Update (u,v)
		if (((nMap[v].g == INF && nMap[u].rhs == INF) || nMap[u].rhs == c_old + nMap[v].g) && u != GOAL_NODE) {
			byte min = INF, j, temp_steps;
			for (j = 0; j < num_neighbors; ++j) {
				temp_steps = nMap[u].actions[j];
				if (temp_steps < INF) {
					byte n_temp = getActionNeighbor(u, j);
					byte temp = INF;
					if (nMap[n_temp].g != INF)
						temp = temp_steps + nMap[n_temp].g;
					if (temp < min)
						min = temp;
				}
			}
			nMap[u].rhs = min;
		}
		updateNode(&nMap[u]);

		// Update (v,u)
		if (((nMap[u].g == INF && nMap[v].rhs == INF) || nMap[v].rhs == c_old + nMap[u].g) && v != GOAL_NODE) {
			byte min = INF, j, temp_steps;
			for (j = 0; j < num_neighbors; ++j) {
				temp_steps = nMap[v].actions[j];
				if (temp_steps < INF) {
					byte n_temp = getActionNeighbor(v, j);
					byte temp = INF;
					if (nMap[n_temp].g != INF)
						temp = temp_steps + nMap[n_temp].g;
					if (temp < min)
						min = temp;
				}
			}
			nMap[v].rhs = min;
		}
		updateNode(&nMap[v]);
	}
	else {
		//Serial.println("Obstacle detected on a node");
		// Obstacle is on a node
		// Need to update all edges to node
		byte neighbor = INF;
		byte steps = 0, i;
		/* Update all nodes affected by the obstacle */

		//nMap[obstacle].g = INF;
		for (i = 0; i < num_neighbors; ++i) {
			if (nMap[obstacle].actions[i] < INF) {

				// Update neighbor Cost
				neighbor = getActionNeighbor(obstacle, i); //getNodeIndex(row - steps, col);
				if (i > 1) {
					c_old = nMap[neighbor].actions[i - 2];
					nMap[neighbor].actions[i - 2] = newCost;
				}
				else {
					c_old = nMap[neighbor].actions[i + 2];
					nMap[neighbor].actions[i + 2] = newCost;
				}

				// Update (neighbor, obstacle)
				if (((nMap[obstacle].g == INF && nMap[neighbor].rhs == INF) || nMap[neighbor].rhs == c_old + nMap[obstacle].g) && neighbor != GOAL_NODE) {
					byte min = INF, j, temp_steps;
					for (j = 0; j < num_neighbors; ++j) {
						temp_steps = nMap[neighbor].actions[j];
						if (temp_steps < INF) {
							byte n_temp = getActionNeighbor(neighbor, j);
							byte temp = INF;
							if (nMap[n_temp].g != INF)
								temp = temp_steps + nMap[n_temp].g;
							if (temp < min)
								min = temp;
						}
					}
					nMap[neighbor].rhs = min;
				}
				updateNode(&nMap[neighbor]);
			}
		}
		// Update Obstacle Cost
		for (i = 0; i < num_neighbors; ++i)
		if (nMap[obstacle].actions[i] != INF)
			nMap[obstacle].actions[i] = newCost;
		nMap[obstacle].rhs = INF;
		updateNode(&nMap[obstacle]);
		//nMap[obstacle].rhs = INF;
		//nMap[obstacle].g = INF;
		//updateNode(&nMap[obstacle]);
	}
}

void DStar::computeShortestPath() {
	byte GOAL = GOAL_NODE;
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
				steps = u->actions[i];
				if (steps < INF) {
					byte n = getActionNeighbor(u_index, i);
					if (n != INF) {
						if (n != GOAL) {
							if (u->g != INF)
								nMap[n].rhs = min(nMap[n].rhs, steps + u->g);
						}
						updateNode(&nMap[n]);
					}
				}
			}
		}
		else {
			byte g_old = u->g;
			u->g = INF;
			
			// For all s in u->neighbors Union 'u' ***** 
			//	rhs(s) = min (for s' in Succ(s) ( cost(s, s') + g(s'))
			for (i = 0; i < num_neighbors; ++i) {
				steps = u->actions[i];
				if (steps < INF) {
					byte n = getActionNeighbor(u_index, i);
					if (n != INF) {
						if ((g_old == INF && nMap[n].rhs == g_old) || nMap[n].rhs == steps + g_old) {
							if (n != GOAL) {
								byte min = INF;
								byte j;
								for (j = 0; j < num_neighbors; ++j) {
									byte steps_temp = nMap[n].actions[j];
									if (steps_temp < INF) {
										byte n_temp = getActionNeighbor(n, j);
										byte rhs_temp = INF;
										if (nMap[n_temp].g != INF)
											rhs_temp = steps_temp + nMap[n_temp].g;
										if (rhs_temp < min)
											min = rhs_temp;
									}
								}
								nMap[n].rhs = min;
							}
						}
						updateNode(&nMap[n]);
					}
				}
			}

			// Now again for 'u'. cost(u, u) == 0
			if (u->rhs == g_old) {
				if (*u != nMap[GOAL_NODE]) {
					byte min = INF;
					for (i = 0; i < num_neighbors; ++i) {
						byte steps_temp = u->actions[i];
						if (steps_temp < INF) {
							byte n_temp = getActionNeighbor(u_index, i);
							byte rhs_temp = INF;
							if (nMap[n_temp].g != INF)
								rhs_temp = steps_temp + nMap[n_temp].g;
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
	byte dist = x*x + y*y;
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
	byte inf = BYTE_INF;
	byte min = inf;
	byte neighbor = inf;
	byte row = nMap[start].row;
	byte col = nMap[start].col;
	byte steps = inf;
	action = inf;
	actionCount = inf;

	byte i, size = 4, minAction = inf, minCount = inf;
	for (i = 0; i < size; ++i) {
		steps = nMap[start].actions[i];
		/*Serial.print("Action: ");
		Serial.print(i);
		Serial.print(" | Edge cost: ");
		Serial.print(steps);
		Serial.println();*/
		if (steps < inf) { // Can go 'up'
			neighbor = getActionNeighbor(start, i);
			byte temp = inf;
			/*Serial.print("G = ");
			Serial.print(nMap[neighbor].g);
			Serial.print(" | rhs = ");
			Serial.print(nMap[neighbor].rhs);
			Serial.println();*/
			if (nMap[neighbor].g != inf)
				temp = steps + nMap[neighbor].g;
			if (temp <= min) {
				min = temp;
				minAction = i;
				minCount = steps;
			}
		}
	}
	action = minAction;
	actionCount = minCount;

	// Update start
	if (action != -1 && actionCount != inf) {
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

void DStar::backtrack(byte row, byte col, byte& action, byte& actionCount) {
	byte offset = 0, node = BYTE_INF;

	// reverse the action
	if (action > 1)
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
				Serial.println("[DStar::addObstacle()] Error: Invalid indexing/action");
				return;
			}
		}
	}
	actionCount = offset;
	start = node;
}

byte DStar::getActionNeighbor(byte node, byte action) {
	byte row = nMap[node].row;
	byte col = nMap[node].col;
	byte steps = nMap[node].actions[action];

	switch (action) {
	case 0: // up
		row = row - steps;
		break;
	case 1: // right
		col = col + steps;
		break;
	case 2: // down
		row = row + steps;
		break;
	case 3: // left
		col = col - steps;
		break;
	default:
		row = BYTE_INF;
		col = BYTE_INF;
	}

	return getNodeIndex(row, col);
}

/* NODE COORDINATES & NEIGHBOR STEP COUNT SETTINGS */
void DStar::initNodes() {
	byte INF = 255;

	// Node #1 - START
	nMap[0].setPos(0, 0);
	nMap[0].setActions(INF, 3, 4, INF); // UP, RIGHT, DOWN, LEFT

	// Node #2
	nMap[1].setPos(0, 3);
	nMap[1].setActions(INF, INF, 2, 3);

	// Node #3
	nMap[2].setPos(2, 2);
	nMap[2].setActions(INF, 1, INF, INF);

	// Node #4
	nMap[3].setPos(2, 3);
	nMap[3].setActions(2, 4, 2, 1);

	// Node #5
	nMap[4].setPos(2, 7);
	nMap[4].setActions(INF, INF, 2, 4);

	// Node #6
	nMap[5].setPos(3, 9);
	nMap[5].setActions(INF, INF, 2, INF);

	// Node #7
	nMap[6].setPos(4, 0);
	nMap[6].setActions(4, 3, 2, INF);

	// Node #8
	nMap[7].setPos(4, 3);
	nMap[7].setActions(2, 4, 4, 3);

	// Node #9
	nMap[8].setPos(4, 7);
	nMap[8].setActions(2, INF, 1, 4);

	// Node #10
	nMap[9].setPos(5, 7);
	nMap[9].setActions(1, 2, 1, INF);

	// Node #11
	nMap[10].setPos(5, 9);
	nMap[10].setActions(2, INF, 4, 2);

	// Node #12
	nMap[11].setPos(6, 0);
	nMap[11].setActions(2, 1, INF, INF);

	// Node #13
	nMap[12].setPos(6, 1);
	nMap[12].setActions(INF, INF, INF, 1);

	// Node #14
	nMap[13].setPos(6, 5);
	nMap[13].setActions(INF, 2, 2, INF);

	// Node #15
	nMap[14].setPos(6, 7);
	nMap[14].setActions(1, INF, INF, 2);

	// Node #16
	nMap[15].setPos(8, 3);
	nMap[15].setActions(4, 2, INF, INF);

	// Node #17
	nMap[16].setPos(8, 5);
	nMap[16].setActions(2, INF, INF, 2);

	// Node #18 - GOAL
	nMap[17].setPos(9, 9);
	nMap[17].setActions(4, INF, INF, INF);
}
