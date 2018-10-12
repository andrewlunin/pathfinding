#include "internaltools.h"

#define INIT_CURRENT_POINT \
int currentPathLength = baseNode.m_PathLength; \
currentPath = baseNode.m_Path; \
currentPoint = currNeighbours[i]; \
baseNode.m_AttachedPoint.SetValue(IMPASSABLE, SET); \
FindTraversableNeighbours(currentPoint, SET); \
baseNode.m_AttachedPoint.SetValue(TRAVERSABLE, SET); \


#define CHECK_NODE_FOR_NEWPATH \
PathIncrease(currentPoint, currentPathLength, currentPath); \
checkResult = CheckInNodes(currentPoint, myNodesMap, oppositeNodesMap, currentPathLength, currentPath, SET); \
if (checkResult == NEWPATH) \
{ \
	if (CutMap(myNodesMap, oppositeNodesMap, SET)) \
	{ \
		if (!currNeighbours.empty()) \
		{ \
			cutNeighbours(currNeighbours, i, SET); \
		} \
		else \
		{ \
			return continueSearching(baseNode, myNodesMap, SET); \
		} \
	} \
	else return STOP_SEARCHING; \
}

using namespace InternalTools;

/*Public interface definition*/

void fillOutBuffer(Settings& SET, int* pOutBuffer)
{
	if (pOutBuffer != nullptr && !PATH1.empty() && !PATH2.empty())
	{
		if (TARGET.m_ArrayPosition == PATH1[0])
		{
			PATH1.pop_back();
			std::reverse(PATH1.begin(), PATH1.end());
			PATH2.reserve(PATH_LENGTH);
			PATH2.insert(PATH2.end(), PATH1.begin(), PATH1.end());
			memmove(pOutBuffer, (int *)&(*PATH2.begin()), PATH_LENGTH * sizeof(int));
		}
		else
		{
			PATH2.pop_back();
			std::reverse(PATH2.begin(), PATH2.end());
			PATH1.reserve(PATH_LENGTH);
			PATH1.insert(PATH1.end(), PATH2.begin(), PATH2.end());
			memmove(pOutBuffer, (int *)&(*PATH1.begin()), PATH_LENGTH * sizeof(int));
		}
	}
}

int FindPath(const int nStartX, const int nStartY,
	const int nTargetX, const int nTargetY,
	const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
	int* pOutBuffer, const int nOutBufferSize)
{
	try
	{
		struct Settings SET(TraversablePoint(nStartX, nStartY, nStartY * nMapWidth + nStartX),
			TraversablePoint(nTargetX, nTargetY, nTargetY * nMapWidth + nTargetX),
			nMapWidth, nMapHeight, nOutBufferSize, pMap);
		if (START == TARGET)
		{
			//exit if start and end points are equal
			return 0;
		}

		//init start maps and points
		NodesMap nodesFromStart;
		NodesMap nodesFromTarget;
		int currentPathLength = 0;
		std::vector<int> currentPath = std::vector<int>();
		currentPath.reserve(MIN_PATH);
		nodesFromStart.AddNode(START, currentPathLength, currentPath);
		nodesFromTarget.AddNode(TARGET, currentPathLength, currentPath);
		nodesFromTarget.GetTopNode().m_Path.push_back(TARGET.m_ArrayPosition);

		//find traversable neighbours of start and final points
		FindTraversableNeighbours(nodesFromTarget.GetTopNode().m_AttachedPoint, SET);
		FindTraversableNeighbours(nodesFromStart.GetTopNode().m_AttachedPoint, SET);

		//main loop of building nodes
		while (!nodesFromStart.Empty() && BuildNodes(nodesFromStart, nodesFromTarget, SET))
		{
			//Nodes from start are built
			if (nodesFromTarget.Empty() || !BuildNodes(nodesFromTarget, nodesFromStart, SET))
			{
				break;
			}
			//Nodes from target are built
		}
		if (PATH_LENGTH == ABOVE_MAX_PATH)
		{
			//if path length is above of max
			return -1;
		}
		else
		{
			if (PATH_LENGTH > nOutBufferSize)
			{
				//if out buffer has no enough space
				std::cout << "Not enough space in output buffer!" << std::endl;
				return -1;
			}
			else
			{
				//fill out buffer and return retVal
				int retVal = PATH_LENGTH;
				fillOutBuffer(SET, pOutBuffer);
				
				//Path finding is complete
				return retVal;
			}
		}
	}
	catch (std::exception e)
	{
		//if exception was thrown return error code and the message
		std::cout << e.what() << std::endl;
		return -1;
	}
}

/*Internal interface definition*/

namespace InternalTools
{
	void FindTraversableNeighbours(TraversablePoint& point, struct Settings& SET)
	{
		int &nodeX = point.m_X, &nodeY = point.m_Y, &arrPos = point.m_ArrayPosition;
		TraversablePoint allNeighbours[MAX_NEIGHBOURS_NUMBER] = { { nodeX, nodeY + 1, arrPos + ROW_LENGTH },
		{ nodeX + 1, nodeY, arrPos + 1 },
		{ nodeX, nodeY - 1, arrPos - ROW_LENGTH },
		{ nodeX - 1, nodeY, arrPos - 1 } };
		for (int i = 0; i < MAX_NEIGHBOURS_NUMBER; ++i)
		{
			if (IsTraversablePoint(allNeighbours[i]))
			{
				point.m_Neighbours.emplace_back(allNeighbours[i]);
			}
		}
	}

	void doRefreshNodes(NodesMap& myNodes, NodesMap& oppositeNodes, struct Settings& SET)
	{
		std::unordered_map<Point, Node, MyHash<Point>>::iterator it = myNodes.GetBeginIt();
		while (it != myNodes.GetEndIt())
		{
			TraversablePoint &currPoint = (it)->second.m_AttachedPoint;
			if (currPoint.m_X >= TOPX || currPoint.m_X < BOTTOMX ||
				currPoint.m_Y >= TOPY || currPoint.m_Y < BOTTOMY)
			{
				currPoint.m_Neighbours.erase(currPoint.m_Neighbours.begin(), currPoint.m_Neighbours.end());
			}
			++it;
		}
		it = oppositeNodes.GetBeginIt();
		while (it != oppositeNodes.GetEndIt())
		{
			TraversablePoint &currPoint = (it)->second.m_AttachedPoint;
			if (currPoint.m_X >= TOPX || currPoint.m_X < BOTTOMX ||
				currPoint.m_Y >= TOPY || currPoint.m_Y < BOTTOMY)
			{
				currPoint.m_Neighbours.erase(currPoint.m_Neighbours.begin(), currPoint.m_Neighbours.end());
			}
			++it;
		}
	}

	bool CutMap(NodesMap& myNodes, NodesMap& oppositeNodes, struct Settings& SET)
	{
		//init new coordinates
		const int modifier = (PATH_LENGTH - SET.m_MinPath) / 2;
		int newTopX = MAX(START.m_X, TARGET.m_X) + modifier;
		int newBottomX = MIN(START.m_X, TARGET.m_X) - modifier;
		int newTopY = MAX(START.m_Y, TARGET.m_Y) + modifier;
		int newBottomY = MIN(START.m_Y, TARGET.m_Y) - modifier;
		if (modifier)
		{
			bool refreshNodes = false;
			if (newTopX < TOPX)
			{
				//if top X needed to be updated
				TOPX = newTopX;
				refreshNodes = true;
			}
			if (newBottomX >= BOTTOMX)
			{
				//if bottom X needed to be updated
				BOTTOMX = newBottomX + 1;
				refreshNodes = true;
			}
			if (newTopY < TOPY)
			{
				//if top Y needed to be updated
				TOPY = newTopY;
				refreshNodes = true;
			}
			if (newBottomY >= BOTTOMY)
			{
				//if bottom Y needed to be updated
				BOTTOMY = newBottomY + 1;
				refreshNodes = true;
			}
			if (refreshNodes)
			{
				//Map is cutted
				doRefreshNodes(myNodes, oppositeNodes, SET);
			}
			return CONTINUE_SEARCHING;
		}
		else
		{
			return STOP_SEARCHING;
		}
	}

	CheckResult CheckInNodes(const TraversablePoint& point, NodesMap& myNodesMap, NodesMap& oppositeNodesMap, const int& pathLength,
		const std::vector<int>& path, struct Settings& SET)
	{
		std::unordered_map<Point, Node, MyHash<Point>>::iterator itFoundNode;
		if (myNodesMap.FindNode(point, itFoundNode))
		{
			//if found node
			Node &foundNode = itFoundNode->second;
			if (pathLength < foundNode.m_PathLength)
			{
				foundNode.m_PathLength = pathLength;
				foundNode.m_Path = path;
			}
			//update neighbours of found node
			foundNode.m_AttachedPoint.m_Neighbours = point.m_Neighbours;
			return MYNODE;
		}
		else
		{
			//if node was not found
			if (oppositeNodesMap.FindNode(point, itFoundNode))
			{
				Node &foundNode = itFoundNode->second;
				int newLength = foundNode.m_PathLength + pathLength;
				if ((newLength) < PATH_LENGTH)
				{
					//if founded path shorter then current
					PATH1 = foundNode.m_Path;
					PATH2 = path;
					PATH_LENGTH = newLength;
					foundNode.m_AttachedPoint.m_Neighbours = point.m_Neighbours;
					return NEWPATH;
				}
				//update neighbours of found node
				foundNode.m_AttachedPoint.m_Neighbours = point.m_Neighbours;
				return OPPOSITENODE;
			}
		}
		return NONODE;
	}

	void PathIncrease(const Point& point, int& pathLength, std::vector<int>& path)
	{
		//Map is cutted
		pathLength += 1;
		path.emplace_back(point.m_ArrayPosition);
	}

	bool inline continueSearching(Node &baseNode, NodesMap& myNodesMap, struct Settings& SET)
	{
		baseNode.m_AttachedPoint.SetValue(IMPASSABLE, SET);
		myNodesMap.EraseTopNode();
		return CONTINUE_SEARCHING;
	}

	void inline cutNeighbours(std::vector<TraversablePoint> &allNeighbours,
		unsigned char currNeighbour,
		struct Settings& SET)
	{
		for (unsigned char k = currNeighbour + 1; k < allNeighbours.size(); ++k)
		{
			if (!IsTraversablePoint(allNeighbours[k]))
			{
				allNeighbours.erase(allNeighbours.begin() + k);
				--k;
			}
		}
	}

	bool BuildNodes(NodesMap& myNodesMap, NodesMap& oppositeNodesMap, struct Settings& SET)
	{
		Node &baseNode = myNodesMap.GetTopNode();
		std::vector<TraversablePoint> &currNeighbours = baseNode.m_AttachedPoint.m_Neighbours;
		std::vector<int> currentPath;
		TraversablePoint currentPoint;
		CheckResult checkResult;
		for (unsigned char i = 0; i < currNeighbours.size(); ++i)
		{
				//init current point
				INIT_CURRENT_POINT
				do
				{
					// choose neighbours count
					switch (currentPoint.m_Neighbours.size())
					{
					case(0):
						CHECK_NODE_FOR_NEWPATH
							currentPoint.SetValue(IMPASSABLE, SET);
						break;
					case (1):
						CHECK_NODE_FOR_NEWPATH
						else if (checkResult == NONODE)
						{
							currentPoint.SetValue(IMPASSABLE, SET);
							currentPoint = currentPoint.m_Neighbours[0]; // first and only neighbour
							FindTraversableNeighbours(currentPoint, SET);
							continue;
						} // if (checkResult == NONODE)
						break;
					case (2):
					case (3):
						CHECK_NODE_FOR_NEWPATH
						else
						{
							if (checkResult == NONODE)
							{
								myNodesMap.AddNode(currentPoint, currentPathLength, currentPath);
							} // if (checkResult == NONODE)
						}
						break;
					} // switch(currentPoint.Neighbours.size())
					break;
				} while (1);
		} // for (int i = 0; i < currNeighbours.size(); ++i)
		baseNode.m_AttachedPoint.SetValue(IMPASSABLE, SET);
		//erase top node from forward path
		myNodesMap.EraseTopNode();
		return CONTINUE_SEARCHING;
	}

	inline NodesMap::~NodesMap()
	{}

	inline void NodesMap::AddNode(const TraversablePoint& attachedPoint, const int& pathLength, const std::vector<int>& path)
	{
		m_nodesQueue.emplace(m_mappedNodes.emplace(std::pair<Point, Node>(attachedPoint, Node(attachedPoint, pathLength, path))).first);
	}

	inline void Point::SetValue(PointValue value, struct Settings& SET)
	{
		MAP[this->m_ArrayPosition] = value;
	}

} // InternalTools


