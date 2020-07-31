#pragma once
#include <queue>
#include <vector>
#include <ctime>
#include <string>

using std::string;


const int VERTEX_SIZE        = 150000;
const int EDGE_SIZE          = 150000;
const int POINT_OF_REFERPATH = 100000;

struct Graph;
struct Vertex;
struct Edge;
struct ReferencePath;
struct CompletePath;
struct TraversingPath;
struct Sequence;
struct Point;
struct Record;

/**
 * The struct of Graph
 * @field edgePos the number of edge in the graph
 * @filed vertexPos The number of vertex in the graph
 * @field edgesVector edge array
 * @field vertexsVector vertex array
 * @field delta the query delta entered by the user
 * @field SECTION_LENGTH interpolation length entered by the user
*/
struct Graph
{
	int     edgePos;
	int     vertexPos;
	Edge*   edgesVector;
	Vertex* vertexsVector;
	double  delta;
	double  SECTION_LENGTH;
};

/**
 * The struct of Vertex
 * @field vertexID ID of this vertex
 * @field minPos minPos related to the reference path
 * @field maxPos maxPos related to the reference path
 * @field parentVertexID used to store the id of the parent node when searching similar to bfs
 * @field referFlag indicate whether the vertex is on the reference path, true if located
 * @field locationFlag indicate whether the vertex is in a safe area, true if located
 * @field accessFlag indicate whether the vertex has been visited, true if visited
 * @field currFlag indicate whether the vertex is on the current path, true if located
 * @field longitude the longitude of the vertex
 * @field latitude the latitude of the vertex
 * @field distToRefer the shortest distance from this vertex to the reference path
 * @field eucLengthToFinalVertex Euclidean distance to the end of the reference path
 * @field traversalOrderWeights weight of each vertex during extension
 * @field adjVertexID the ID of the vertex adjacent to the vertex
 * @field adjEdgeID the IDof the edge adjacent to the vertex
*/
struct Vertex
{
	int    vertexID;
	int    minPos;
	int    maxPos;
	int    parentVertexID;
	bool   referFlag;
	bool   locationFlag;
	bool   accessFlag;
	bool   currFlag;
	double longitude;
	double latitude;
	double distToRefer;
	double eucLengthToFinalVertex;
	double traversalOrderWeights;
	std::vector<int> adjVertexID;
	std::vector<int> adjEdgeID;
};

/**
 * The struct of Edge
 * @field edgeID ID of this edge
 * @field firstVertexID the ID of the first vertex of the edge
 * @field secVertexID the ID of the second vertex of the edge
 * @field length Euclidean distance between the two vertices of the edge
 * @field realLength the real distance between the two vertices of the edge
*/
struct Edge
{
	int    edgeID;
	int    firstVertexID;
	int    secVertexID;
	double length;
	double realLength;
};

/**
 * The struct of ReferencePath
 * @field initialVertex start vertex of reference path
 * @field finalVertex stop vertex of reference path
 * @field numOfReferPoint number of trajectory points after reference path interpolation
 * @field referDFDFlag calculated sign at the end of the reference path
 * @field pointOfRefer trajectory point array
 * @field referenceLength the total length of the reference path
 * @field edges included edges of the reference path
*/
struct ReferencePath
{
	int    initialVertex;
	int    finalVertex;
	int    numOfReferPoint;
	bool   referDFDFlag;
	Point* pointOfRefer;
	double referenceLength;
	std::vector<Edge> edges;
};

/**
 * The struct of CompletePath
 * @field initialVertex start vertex of complete path
 * @field finalVertex stop vertex of complete path
 * @field completeLength the total length of the complete path
 * @field edges included edges of the complete path
*/
struct CompletePath
{
	int    initialVertex;
	int    finalVertex;
	double completeLength;
	std::vector<Edge> edges;
};

/**
 * The struct of TraversingPath
 * @field minPos minPos of the traversing path
 * @filed maxPos maxPos of the traversing path
 * @field finalVertex end vertex ID of the traversing path
 * @field traversingPathID ID of the traversing path
 * @field distToRefer the distance from the traversing path to the reference path
 * @field eucDelta lb of the path
 * @field eucDeltaOfTrue true lb of the path
 * @field edges included edges of the traversing path
*/
struct TraversingPath
{
	int    minPos;
	int    maxPos;
	int    finalVertexID;
	int    traversingPathID;
	double distToRefer;
	double eucDelta;
	double eucDeltaOfTrue;
	std::vector<int> edges;
	bool operator<(const TraversingPath path) const
	{
		return this->eucDelta > path.eucDelta;
	}
};

/**
 * The struct of Sequence
 * @field startPos start position
 * @field stopPos stop position
 * @field seqFlag increase or decrease sign in the interval, increase is true
*/
struct Sequence
{
	int  startPos;
	int  stopPos;
	bool seqFlag;
};

/**
 * The struct of Point
 * @field x the longitude value of the point
 * @field y The latitude value of the point
*/
struct Point
{
	double x;
	double y;
};

/**
 * The struct of Record
 * @field numDFDCal calculation times of DFD
 * @field numTemDiscrete number of trajectory points after current path discretization
 * @field numReferDiscrete number of trajectory points after reference path discretization
 * @field locationOfRefer when the final DFD is obtained, the position of the discrete point on the reference path
 * @field numOfOutPriQueue number of dequeues at the time of inquery
 * @field comPathSize the edge size of the complete path
 * @field referPathSize the edge size of the reference path
 * @field eucConQueryPos auxiliary variables used in continuous query of Euclidean space
 * @field runTime the run time for this query
 * @field currPathLength the length of the current path
 * @field comPathLength the length of the complete path
 * @field ratioNumEdge the ratio of the edge size of the current path to the complete path
 * @field ratioPathLength the ratio of the length of the current path to the complete path
 * @field referPathLength the length of the reference path
*/
struct Record
{
	int    numDFDCal;
	int    numTemDiscrete;
	int    numReferDiscrete;
	int    locationOfRefer;
	int    numOfOutPriQueue;
	int    comPathSize;
	int    referPathSize;
	int    eucConQueryPos;
	double runTime;
	double currPathLength;
	double comPathLength;
	double ratioNumEdge;
	double ratioPathLength;
	double referPathLength;
};