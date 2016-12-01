#include "GridPathPlanner.h"

GridPathPlanner::GridPathPlanner(PartiallyKnownGrid* grid, bool use_adaptive_a_star) 
{
	useAdaptive = use_adaptive_a_star;
	numExpansions = 0;
	
	mGrid = grid;
	mRows = grid->GetHeight();
	mCols = grid->GetWidth();
	goal = grid->GetGoalLocation();
	start = grid->GetCurrentLocation();

	// Dynamic sized array
	allNodes = new Node*[mRows];
	for(int i = 0; i < mRows; i++)
	{
    	allNodes[i] = new Node[mCols];
	}

	// Init starting values for all nodes
	for (int y = 0; y < mRows; y++) 
	{
		for (int x = 0; x < mCols; x++) 
		{
			Node n;
			n.loc = xyLoc(x,y);
			n.visited = false;

			// Heuristic calculation - Manhattan Distance
			n.h = GetHeuristic( n.loc, goal );
			//std::cout << "INIT Node: " << n.loc.y << ", " << n.loc.x << " | h=" << n.h << std::endl;

			if( n.loc == start )
			{
				n.g = 0;
				n.f = n.h;
				std::cout << "Start Node: " << n.loc.y << ", " << n.loc.x << std::endl;
				openList.insert(n);
			}
			else
			{
				n.g = std::numeric_limits<float>::max();
				n.f = std::numeric_limits<float>::max();
			}
			allNodes[y][x] = n;
		}
	}
}

GridPathPlanner::~GridPathPlanner()
{
	// Deallocated array
	for(int i = 0; i < mCols; i++)
	{
		delete [] allNodes[i];
	}
	delete [] allNodes;
}

// Returns the cell that the agent should move to next when following 
// a shortest path from its current cell to the target cell
xyLoc GridPathPlanner::GetNextMove(PartiallyKnownGrid* grid) 
{
	if( !grid->GoalReached() )
	{
		mGrid = grid;
		numExpansions++;
		AStar();

		xyLoc next = path.back().loc;
		path.pop_back();

		std::cout << "MOVE TO: " << next.y << ", " << next.x << std::endl;

		return next;
	}
	std::cout << "STILL ALIVE" << std::endl;
	return kInvalidXYLoc;
}

float GridPathPlanner::GetHeuristic( xyLoc cur, xyLoc goal ) 
{
	return std::abs( goal.x - cur.x ) + std::abs( goal.y - cur.y );
}

// NOTE: The node are indexed by col, row
void GridPathPlanner::AStar()
{
	//std::cout << "A STAR" << std::endl;
	start = mGrid->GetCurrentLocation();
	Node startNode = allNodes[start.y][start.x];
	openList.insert(startNode);

	if( useAdaptive )
	{
		// Adaptive A*
	}
	else
	{
		// Forward A*
		while( !openList.empty() )
		{
			std::set<Node>::iterator it = openList.begin();
			openList.erase(it);
			Node cur = allNodes[(*it).loc.y][(*it).loc.x];
			xyLoc cLoc = cur.loc;
			allNodes[cLoc.y][cLoc.x].visited = true;

			if( cur.successor != NULL )
			{
				std::cout << "CURRENT Node: " << cLoc.y << ", " << cLoc.x << " | Successor is " << cur.successor->loc.y << ", " << cur.successor->loc.x <<
				" | g=" << cur.g << " | h=" << cur.h << " | f=" << cur.f << " | Visited: " << cur.visited << std::endl;
			}

			// Determine neighbors
			std::vector<xyLoc> neighbors;
			neighbors.push_back(xyLoc(cLoc.x+1, cLoc.y));
			neighbors.push_back(xyLoc(cLoc.x-1, cLoc.y));
			neighbors.push_back(xyLoc(cLoc.x, cLoc.y+1));
			neighbors.push_back(xyLoc(cLoc.x, cLoc.y-1));

			for ( int i = 0; i < neighbors.size(); i++ ) 
			{
				xyLoc adj = neighbors[i];
				Node n = allNodes[adj.y][adj.x];

				// Check if valid: not blocked or already visited
				if ( mGrid->IsValidLocation(adj) && !mGrid->IsBlocked(adj) && !n.visited ) 
				{
					Node n = allNodes[adj.y][adj.x];
					n.g = cur.g + 1.0f;
					n.f = n.g + n.h;
					n.successor = &allNodes[cLoc.y][cLoc.x];
					allNodes[adj.y][adj.x] = n;

					std::cout << "NEIGHBOR Node: " << adj.y << ", " << adj.x << " | Successor is " << n.successor->loc.y << ", " << n.successor->loc.x << std::endl;
					std::cout << "g=" << n.g << " | h=" << n.h << " | f=" << n.f << " | Visited: " << n.visited << std::endl;
					openList.insert(n);
					std::cout << "OPEN LIST SIZE: " << openList.size() << std::endl;
				}
			}
		}
	}

	// Build the path from start
	path.clear();
	xyLoc pos = start;
	Node n = allNodes[pos.y][pos.x];
	path.push_back(n);
	std::cout << "HERE" << std::endl;
	std::cout << "START Node: " << pos.y << ", " << pos.x << " | g=" << n.g << " | h=" << n.h << " | f=" << n.f << " |  Successor: " << n.successor->loc.y << ", " << n.successor->loc.x << std::endl;

	while ( n.successor == NULL )
	{
		n = *n.successor;
		pos = n.loc;
		path.push_back(n);
		std::cout << "PATH: " << n.loc.y << ", " << n.loc.x << std::endl;
	}
}

// Returns number of states expanded by the most recent search
int GridPathPlanner::GetNumExpansions() 
{
	return numExpansions;
}
