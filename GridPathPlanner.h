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
	xyLoc AStar();
	xyLoc ForwardA();
	xyLoc AdaptiveA();
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
	PartiallyKnownGrid* mGrid;
	int mRows, mCols;
	int numExpansions;
	bool useAdaptive;
	std::vector<Node*> open, closed;	
};

#endif
