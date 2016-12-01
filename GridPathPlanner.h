#ifndef GRID_PATH_PLANNER_H
#define GRID_PATH_PLANNER_H

#include "PartiallyKnownGrid.h"
#include <vector>
#include <set>
#include <iostream>
#include <cmath>
#include <limits>

class GridPathPlanner{
public:
	GridPathPlanner(PartiallyKnownGrid* grid, bool use_adaptive_a_star = false);
	~GridPathPlanner();

	xyLoc GetNextMove(PartiallyKnownGrid* grid);
	float GetHeuristic( xyLoc cur, xyLoc goal );
	void AStar();
	int GetNumExpansions();

private:
	PartiallyKnownGrid* mGrid;
	int mRows, mCols;
	int numExpansions;
	xyLoc goal, start;
	bool useAdaptive;

	typedef struct Node
	{
		Node *successor;
		float g;	// shortest path from this node to start node
		float h; 	// heuristic: goal (manhattan) dist of this node  
		float f;	// f = g + h
		bool visited;
		xyLoc loc;
	} Node;

	// Comparison function using tie-breaking rules
	struct CompareNodes 
	{
		bool operator() (Node const & n1, Node const & n2) 
		{
			if( n1.f != n2.f )
				return n1.f < n2.f;
			else if( n1.g != n2.g )
				return n1.g > n2.g;
			else
				return n1.loc < n2.loc;
		}
	};

	Node **allNodes;
	std::set<Node, CompareNodes> openList;
	std::vector<Node> path;
};

#endif
