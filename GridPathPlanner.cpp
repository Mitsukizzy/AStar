#include "GridPathPlanner.h"

GridPathPlanner::GridPathPlanner(PartiallyKnownGrid* grid, bool use_adaptive_a_star) 
{
	useAdaptive = use_adaptive_a_star;
	numExpansions = 0;
	
	mGrid = grid;
	mRows = grid->GetHeight();
	mCols = grid->GetWidth();

	// Allocate space for dynamically sized array
	// allNodes = new Node*[mRows];
	// for(int i = 0; i < mRows; i++)
	// {
    // 	allNodes[i] = new Node[mCols];
	// }
	// InitAllNodes();
}

GridPathPlanner::~GridPathPlanner()
{
	// // Deallocated array
	// for(int i = 0; i < mCols; i++)
	// {
	// 	delete [] allNodes[i];
	// }
	// delete [] allNodes;
}

// Returns the cell that the agent should move to next when following 
// a shortest path from its current cell to the target cell
xyLoc GridPathPlanner::GetNextMove(PartiallyKnownGrid* grid) 
{
	if( !grid->GoalReached() )
	{
		mGrid = grid;
		numExpansions++;
		
		xyLoc next = AStar();
		return next;
	}
	return kInvalidXYLoc;
}

float GridPathPlanner::GetHeuristic( xyLoc cur, xyLoc goal ) 
{
	return std::abs( goal.x - cur.x ) + std::abs( goal.y - cur.y );
}

// NOTE: The node are indexed by col, row
xyLoc GridPathPlanner::AStar()
{
	// Reset the all the nodes using current location as start
	open.clear();
	closed.clear();
	Node *startNode = new Node();
	startNode->loc = mGrid->GetCurrentLocation();
	startNode->g = 0;
	startNode->h = GetHeuristic( startNode->loc, mGrid->GetGoalLocation() );
	startNode->f = startNode->h + startNode->g;
	open.push_back(startNode);

	if( useAdaptive )
	{
		return AdaptiveA();
	}
	else
	{
		return ForwardA();
	}
}

xyLoc GridPathPlanner::ForwardA()
{
	int count = 5;
	while( !open.empty() )
	{
		Node *cur = open.front();

		if( cur->loc == mGrid->GetGoalLocation() )
		{
			// Retrace back to find next move
			Node *next = cur->parent;
			while( next->parent )
			{
				cur = next;
				next = next->parent;
			}
			return cur->loc;
		}
		closed.push_back(cur);
		open.erase(open.begin());

		std::vector<xyLoc> neighbors;
		neighbors.push_back(xyLoc(cur->loc.x+1, cur->loc.y));
		neighbors.push_back(xyLoc(cur->loc.x-1, cur->loc.y));
		neighbors.push_back(xyLoc(cur->loc.x, cur->loc.y+1));
		neighbors.push_back(xyLoc(cur->loc.x, cur->loc.y-1));

		for ( int i = 0; i < neighbors.size(); i++ ) 
		{
			Node *n = new Node();
			n->loc = neighbors[i];		

			if ( mGrid->IsValidLocation(n->loc) && !mGrid->IsBlocked(n->loc) ) 
			{
				// Check if neighbor is in closed list and open list
				bool isInClosed = false;
				bool isInOpen = false;
				for( std::vector<Node*>::iterator it = closed.begin(); it != closed.end(); ++it)
				{
					if( n->loc == (*it)->loc )
						isInClosed = true;
				}
				for( std::vector<Node*>::iterator it = open.begin(); it != open.end(); ++it)
				{
					if( n->loc == (*it)->loc )
						isInOpen = true;
				}

				// Check if neighbor is in open list
				if( !isInClosed && !isInOpen )
				{
					n->g = cur->g + 1;
					n->h = GetHeuristic( n->loc, mGrid->GetGoalLocation() );
					n->f = n->g + n->f;
					n->parent = cur;
					open.push_back(n);
					std::sort( open.begin(), open.end(), CompareNodes() );
				}
				
				if( cur->g + 1 < n->g )
				{				
					n->g = cur->g + 1;
					n->f = n->g + n->h;
					n->parent = cur;
				}
			}
		}
	}
}

xyLoc GridPathPlanner::AdaptiveA()
{



}

// Returns number of states expanded by the most recent search
int GridPathPlanner::GetNumExpansions() 
{
	return numExpansions;
}
