#ifndef GRID_PATH_PLANNER_H
#define GRID_PATH_PLANNER_H

#include "PartiallyKnownGrid.h"
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>

class GridPathPlanner{
public:
	GridPathPlanner(PartiallyKnownGrid* grid, bool use_adaptive_a_star = false);
	~GridPathPlanner();

	xyLoc GetNextMove(PartiallyKnownGrid* grid);
	float GetHeuristic( xyLoc cur, xyLoc goal );
	int GetNumExpansions();

	typedef struct Node
	{
		Node *parent;
		int g;	// shortest path from this node to start node
		int h; 	// heuristic: goal (manhattan) dist of this node  
		int f;	// f = g + h
		xyLoc loc;

		bool operator == (const Node* & n) const
		{
			std::cout << loc.x << ", " << loc.y << std::endl;
			return loc == n->loc;
		}
	} Node;

	// Comparison function using tie-breaking rules
	struct CompareNodes 
	{
		bool operator() (Node* const & n1, Node* const & n2) 
		{
			if( n1->f != n2->f )
				return n1->f < n2->f;
			else if( n1->g != n2->g )
				return n1->g > n2->g;
			else
				return n1->loc < n2->loc;
		}
	};

private:
	int numExpansions;
	int mRows, mCols;
	bool useAdaptive;
	std::vector<Node*> open, closed;	
	int **hvalues;
};

#endif
