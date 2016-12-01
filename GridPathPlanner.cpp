#include "GridPathPlanner.h"

GridPathPlanner::GridPathPlanner(PartiallyKnownGrid* grid, bool use_adaptive_a_star) 
{
	useAdaptive = use_adaptive_a_star;
	numExpansions = 0;
	mRows = grid->GetHeight();
	mCols = grid->GetWidth();

	// Init H values
	hvalues = new int*[mRows];
 	for(int i = 0; i < mRows; i++)
	{
     	hvalues[i] = new int[mCols];
 	}

	for (int y = 0; y < mRows; y++) 
 	{
 		for (int x = 0; x < mCols; x++) 
		{
			hvalues[y][x] = GetHeuristic( xyLoc(x,y), grid->GetGoalLocation() );
		}
	 }
}

GridPathPlanner::~GridPathPlanner()
{
	// Deallocate array
 	for(int i = 0; i < mRows; i++)
 	{
 		delete [] hvalues[i];
 	}
 	delete [] hvalues;
}

// Returns the cell that the agent should move to next when following 
// a shortest path from its current cell to the target cell
xyLoc GridPathPlanner::GetNextMove(PartiallyKnownGrid* grid) 
{
	// Reset the all the lists 
	open.clear();
	closed.clear();

	// Use current location as start
	Node *startNode = new Node();
	startNode->loc = grid->GetCurrentLocation();
	startNode->g = 0;
	startNode->h = hvalues[startNode->loc.y][startNode->loc.x];
	startNode->f = startNode->h + startNode->g;
	open.push_back(startNode);

	while( !open.empty() )
	{
		Node *cur = open.front();

		if( cur->loc == grid->GetGoalLocation() )
		{
			// Save goal node in member variable			
			Node* goal = cur;

			// Retrace back to find next move
			Node *next = cur->parent;
			while( next->parent )
			{
				cur = next;
				next = next->parent;
			}
			std::cout << closed.size() << std::endl;
			numExpansions += closed.size();

			if( useAdaptive )
			{
				for( std::vector<Node*>::iterator it = closed.begin(); it != closed.end(); ++it )
				{
					xyLoc hLoc = (*it)->loc;
					hvalues[hLoc.y][hLoc.x] = goal->g - (*it)->g;
				}
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

			if ( grid->IsValidLocation(n->loc) && !grid->IsBlocked(n->loc) ) 
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
					n->h = hvalues[n->loc.y][n->loc.x];
					n->g = cur->g + 1;
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

float GridPathPlanner::GetHeuristic( xyLoc cur, xyLoc goal ) 
{
	return std::abs( goal.x - cur.x ) + std::abs( goal.y - cur.y );
}

// Returns number of states expanded by the most recent search
int GridPathPlanner::GetNumExpansions() 
{
	return numExpansions;
}
