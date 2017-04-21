For Arduino setup:
	1) Copy the 'PathFinding' directory
	2) Paste in Arduino Installation libraries folder
		e.g., C:\Program Files (x86)\Arduino\libraries

Path Finding speudo code:

Car -> has a current orientation (UP, RIGHT, DOWN, LEFT) and a ROW & COL position

Init pathfinding
computeShortestPath()
getNextAction(action, actionCount)
	copies the desired action (UP, RIGHT, DOWN, LEFT) and the number of times to take that action
	each action moves one 1x1 ft. square
if (obstacle)
	recomputeShortestPath(car->row, car->col, action, actionCount)