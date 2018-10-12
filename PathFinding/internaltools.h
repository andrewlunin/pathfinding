#ifndef INTERNAL_H
#define INTERNAL_H

#include <vector>
#include <iostream>
#include <exception>
#include <cstring>
#include <unordered_map>
#include <algorithm>
#include <queue>
#include <ctime>

#define MIN(x,y) ((x < y) ? x : y)
#define MAX(x,y) ((x > y) ? x : y)
#define SET CurrentSettings
#define TOPX SET.m_MapTopX
#define TOPY SET.m_MapTopY
#define BOTTOMX SET.m_MapBottomX
#define BOTTOMY SET.m_MapBottomY
#define MAP SET.m_Map
#define START SET.m_StartPoint
#define TARGET SET.m_TargetPoint
#define OUTBUFFSIZE SET.m_OutBufferSize
#define PATH1 SET.m_PathPart1
#define PATH2 SET.m_PathPart2
#define PATH_LENGTH SET.m_PathLength
#define ROW_LENGTH SET.m_RowLength
#define COLUMN_HEIGHT SET.m_ColumnHeight
#define MAX_NEIGHBOURS_NUMBER 4
#define ABOVE_MAX_PATH (ROW_LENGTH * COLUMN_HEIGHT)
#define MAX_PATH ((COLUMN_HEIGHT % 2) ? ((((COLUMN_HEIGHT + 1) * ROW_LENGTH) / 2) + (COLUMN_HEIGHT / 2)) : ((((ROW_LENGTH + 1) * COLUMN_HEIGHT) / 2) + (ROW_LENGTH / 2)))
#define Distance(pointA, pointB) (abs(pointA.m_X - pointB.m_X) + abs(pointA.m_Y - pointB.m_Y))
#define MIN_PATH Distance(TARGET,START)
#define MapValue(point) (((point.m_Y >= BOTTOMY) && (point.m_X >= BOTTOMX) && (point.m_X < TOPX) && (point.m_Y < TOPY)) ? MAP[ point.m_ArrayPosition ] : IMPASSABLE)
#define IsTraversablePoint(point) ( MapValue(point) )
#define Sign(x) ((x <= 0) ? -1 : 1)


namespace InternalTools
{
	/*Internal types*/

	typedef enum {
		IMPASSABLE,
		TRAVERSABLE
	} PointValue;

	enum {
		STOP_SEARCHING = false,
		CONTINUE_SEARCHING = true
	};

	typedef enum {
		NONODE = 0,
		MYNODE = 1,
		OPPOSITENODE = 2,
		NEWPATH = 3
	} CheckResult;

	struct Point
	{
		int m_X;
		int m_Y;
		int m_ArrayPosition;
		Point(const int x = 0, const int y = 0, const int arrPos = 0) :
			m_X(x),
			m_Y(y),
			m_ArrayPosition(arrPos)
		{}
		Point(const Point& rhs) :
			m_X(rhs.m_X),
			m_Y(rhs.m_Y),
			m_ArrayPosition(rhs.m_ArrayPosition)
		{}
		bool operator==(const Point& rhs) const
		{
			return (this->m_ArrayPosition == rhs.m_ArrayPosition);
		}
		Point& operator=(const Point& rhs)
		{
			this->m_X = rhs.m_X;
			this->m_Y = rhs.m_Y;
			this->m_ArrayPosition = rhs.m_ArrayPosition;
			return *(this);
		}
		bool operator< (const Point& rhs) const
		{
			return (this->m_ArrayPosition < rhs.m_ArrayPosition);
		}
		void SetValue(PointValue rhs, struct Settings& SET);
	};

	struct TraversablePoint : Point
	{
		std::vector<TraversablePoint> m_Neighbours;
		TraversablePoint(const int x = 0, const int y = 0, const int arrPos = 0) :
			Point(x, y, arrPos)
		{}
		TraversablePoint(const TraversablePoint& rhs) :
			Point(rhs),
			m_Neighbours(rhs.m_Neighbours)
		{}
		bool operator==(const TraversablePoint& rhs) const
		{
			return Point::operator==(rhs);
		}
		TraversablePoint& operator=(const TraversablePoint& rhs)
		{
			Point::operator=(rhs);
			this->m_Neighbours = rhs.m_Neighbours;
			return *(this);
		}
	};

	struct Node
	{
		TraversablePoint m_AttachedPoint;
		int m_PathLength;
		std::vector<int> m_Path;
		Node(const TraversablePoint& attachedPoint, const int& pathLengt, const std::vector<int>&  path) :
			m_AttachedPoint(attachedPoint),
			m_PathLength(pathLengt),
			m_Path(path)
		{}
		bool operator==(const Node& rhs)
		{
			return (this->m_AttachedPoint == rhs.m_AttachedPoint);
		}
		bool operator==(const TraversablePoint& rhs)
		{
			return (this->m_AttachedPoint == rhs);
		}
	};

	template <class T>
	class MyHash;

	template<>
	class MyHash<Point>
	{
	public:
		std::size_t operator()(Point const& point) const
		{
			return std::hash<int>()(point.m_ArrayPosition);
		}
	};

	class NodesComparer
	{
	public:
		bool operator()(const std::unordered_map<Point, Node, MyHash<Point>>::iterator& nodeA, const std::unordered_map<Point, Node, MyHash<Point>>::iterator& nodeB)
		{
			return (nodeA->second.m_PathLength > nodeB->second.m_PathLength);
		}
	};

	class NodesMap
	{
	private:
		std::priority_queue < std::unordered_map<Point, Node, MyHash<Point>>::iterator,
			std::vector<std::unordered_map<Point, Node, MyHash<Point>>::iterator>, NodesComparer > m_nodesQueue;
		std::unordered_map<Point, Node, MyHash<Point>> m_mappedNodes;
	public:
		NodesMap()
		{}
		bool Empty()
		{
			return m_mappedNodes.empty();
		}
		Node& GetTopNode()
		{
			return m_nodesQueue.top()->second;
		}
		void EraseTopNode()
		{
			std::unordered_map<Point, Node, MyHash<Point>>::iterator tmp = m_nodesQueue.top();
			m_nodesQueue.pop();
			m_mappedNodes.erase(tmp);
		}
		bool FindNode(const Point& point,
			std::unordered_map<Point, Node, MyHash<Point>>::iterator& foundNode)
		{
			foundNode = m_mappedNodes.find(point);
			if (foundNode != m_mappedNodes.end())
				return true;
			else
				return false;
		}
		std::unordered_map<Point, Node, MyHash<Point>>::iterator GetBeginIt()
		{
			return m_mappedNodes.begin();
		}
		std::unordered_map<Point, Node, MyHash<Point>>::iterator GetEndIt()
		{
			return m_mappedNodes.end();
		}
		~NodesMap();
		void AddNode(const TraversablePoint& attachedPoint, const int& pathLengt, const std::vector<int>& path);
	};

	struct Settings
	{
	public:
		const TraversablePoint m_StartPoint;
		const TraversablePoint m_TargetPoint;
		const int m_OutBufferSize;
		int m_MapTopX, m_MapTopY, m_MapBottomX, m_MapBottomY;
		const int m_RowLength, m_ColumnHeight;
		unsigned char* m_Map;
		std::vector<int> m_PathPart1;
		std::vector<int> m_PathPart2;
		int m_PathLength, m_MinPath;
		Settings(const TraversablePoint startPoint, const TraversablePoint targetPoint,
			const int mapWidth, const int mapHeight, const int outBufferSize, const unsigned char* map) :
			m_ColumnHeight(mapHeight),
			m_RowLength(mapWidth),
			m_OutBufferSize(outBufferSize),
			m_StartPoint(startPoint),
			m_TargetPoint(targetPoint),
			m_PathLength(m_ColumnHeight * m_RowLength),
			m_MinPath(Distance(startPoint, targetPoint)),
			m_MapTopX(mapWidth),
			m_MapTopY(mapHeight),
			m_MapBottomX(0),
			m_MapBottomY(0)
		{
			m_Map = new unsigned char[(m_ColumnHeight * m_RowLength)];
			m_PathPart1 = std::vector<int>();
			m_PathPart2 = std::vector<int>();
			try
			{
				if (m_Map != nullptr && map != nullptr)
				{
					memmove(this->m_Map, map, (m_ColumnHeight * m_RowLength) * sizeof(char));
				}
			}
			catch (std::exception& e)
			{
				delete m_Map;
				throw e;
			}
		}
		~Settings()
		{
			delete[] m_Map;
		}
	};

	// Internal types

	/*Internal interface*/

	/*Find traversable neighbours near specified point*/
	inline void FindTraversableNeighbours(TraversablePoint& point, struct Settings& SET);

	/*Cut the map relatively current minimal path*/
	inline bool CutMap(NodesMap& myNodes, NodesMap& oppositeNodes, struct Settings& SET);

	/*Find node with specified attached point and return the result. If such node exists, compare its path with specified as input parameter path.*/
	inline CheckResult CheckInNodes(const TraversablePoint& point, NodesMap& myNodesMap, NodesMap& oppositeNodesMap, const int& pathLength,
		const std::vector<int>& path, struct Settings& SET);

	/*Build nodes from first element in myNodes vector. Finally, erase this element.*/
	inline bool BuildNodes(NodesMap& myNodesMap, NodesMap& oppositeNodesMap, struct Settings& SET);

	/*Increase pathLength and add point position to path vector.*/
	inline void PathIncrease(const Point& point, int& pathLength, std::vector<int>& path);

	// Internal interface
}

#endif
