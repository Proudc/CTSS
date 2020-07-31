#include <float.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>

#include "../../include/ContinuousQuery/Common.h"
#include "../../include/ContinuousQuery/Discrete.h"
#include "../../include/ContinuousQuery/Frechet.h"

namespace
{

/**
 * Calculate the Euclidean distance between two points
 * @param pt1 first point
 * @param pt2 second point
 * @return the Euclidean distance between two points
*/
double euc(Point pt1, Point pt2)
{
	double ux, uy;
	ux = pt1.x - pt2.x;
	uy = pt1.y - pt2.y;
	return sqrt(ux * ux + uy * uy);
}

/**
 * Calculate the maximum value between two points
 * @param a first value
 * @param b second value
 * @param the maximum value between two points
*/
double max(double a, double b)
{
	return a > b ? a : b;
}

/**
 * Calculate the minimum value between three points
 * @param a first value
 * @param b second value
 * @param c third value
 * @param the minimum value between three points
*/
double min(double a, double b, double c)
{
	double m;
	m = a < b ? a : b;
	return m < c ? m : c;
}

/**
 * Recursively calculate DFD
 * @param ca auxiliary array
 * @param i the first dimension of the array
 * @param j the second dimension of the array
 * @param p first trajectory
 * @param q second trajectory
 * @return DFD between two trajectories
*/
double dist(std::vector<std::vector<double>>& ca, int i, int j, Point* p, Point* q)
{
	if (ca[i][j] > -1)
	{
		return ca[i][j];
	}
	else if (i == 0 && j == 0)
	{
		ca[i][j] = euc(p[0], q[0]);
	}
	else if (i > 0 && j == 0)
	{
		ca[i][j] = max(dist(ca, i - 1, 0, p, q), euc(p[i], q[0]));
	}
	else if (i == 0 && j > 0)
	{
		ca[i][j] = max(dist(ca, 0, j - 1, p, q), euc(p[0], q[j]));
	}
	else if (i > 0 && j > 0)
	{
		ca[i][j] = max(
			min(
				dist(ca, i - 1, j, p, q),
				dist(ca, i - 1, j - 1, p, q),
				dist(ca, i, j - 1, p, q)),
			euc(p[i], q[j]));
	}
	else
	{
		ca[i][j] = DBL_MAX;
	}
	return ca[i][j];
}


}

/**
 * Calculate the dfd of the two trajectories
 * @param m the length of the first trajectory
 * @param n the length of the second trajectory
 * @param p the first trajectory
 * @param q the senond trajectory
 * @return the dfd of the two trajectories
*/
double dfdDist(const int m, const int n, Point* p, Point* q)
{
	std::vector<std::vector<double>> ca(m, std::vector<double>(n, -1));
	double temDist = dist(ca, m - 1, n - 1, p, q);
	return temDist;
}

/**
 * Expansion of dfd calculation method
 * Calculate the dfd between traversing path and reference path
 * Call dfdDist() at the bottom
 * @param myGraph pointer to a graph
 * @param temPath pointer to a traversing path, which intersects with referencepath
 * @param referencePath pointer to a reference path
 * @param lastPath the complete path after filling the second half of the traversing path, this variable is the return value 
 * @param finalVertexID intersection point of traversing path and reference path
 * @return the dfd between traversing path and reference path
*/
double dfdDistBetweenTwoPaths(Graph* myGraph, TraversingPath* temPath, ReferencePath* referencePath, std::vector<Edge>& lastPath, int finalVertexID)
{
	// printf("Start calculating the dfd between a half path and the reference path...\n");
	std::vector<Edge> finalPath;
	for (unsigned int i = 0; i < temPath->edges.size(); i++)
	{
		finalPath.push_back(myGraph->edgesVector[temPath->edges[i]]);
	}
	int pos;
	for (int i = (referencePath->edges.size()) - 1; i >= 0; i--)
	{
		int firstVertexID = referencePath->edges[i].firstVertexID;
		int secVertexID   = referencePath->edges[i].secVertexID;
		if ((firstVertexID == finalVertexID) || (secVertexID == finalVertexID))
		{
			pos = i;
			break;
		}
	}
	for (unsigned int i = pos; i < referencePath->edges.size(); i++)
	{
		finalPath.push_back(referencePath->edges[i]);
	}

	lastPath = finalPath;

	std::vector<Point> temPointList = creatTrajPoint(myGraph, finalPath, referencePath->initialVertex);
	Point p[temPointList.size()];
	for (unsigned int i = 0; i < temPointList.size(); i++)
	{
		p[i].x = temPointList[i].x;
		p[i].y = temPointList[i].y;
	}
	double temDist = dfdDist(temPointList.size(), referencePath->numOfReferPoint, p, referencePath->pointOfRefer);
	return temDist;
}

/**
 * Calculate the dfd of the two path
 * Call dfdDist() at the bottom
 * @param myGraph pointer to a graph
 * @param finalPath a complete path that can be calculated
 * @param referencePath pointer to a reference path
 * @return the dfd of the two path
*/
double DFD(Graph* myGraph, std::vector<Edge> finalPath, ReferencePath* referencePath)
{
	std::vector<Point> temPointList;
	temPointList = creatTrajPoint(myGraph, finalPath, referencePath->initialVertex);
	Point p[temPointList.size()];
	for (unsigned int i = 0; i < temPointList.size(); i++)
	{
		p[i].x = temPointList[i].x;
		p[i].y = temPointList[i].y;
	}
	double temDist = dfdDist(temPointList.size(), referencePath->numOfReferPoint, p, referencePath->pointOfRefer);
	return temDist;
}

/**
 * Calculate the dfd of the two small path
 * Call dfdDist() at the bottom
 * @param myGraph pointer to a graph
 * @param firstPath first small path
 * @param secPath second small path
 * @param initialVertex the common starting point of the two paths
 * @return the dfd of the two small path
*/
double dfdBetweenTwoLocalPaths(Graph* myGraph, std::vector<Edge>& firstPath, std::vector<Edge>& secPath, int initialVertex)
{
	std::vector<Point> firstPointList;
	std::vector<Point> secPointList;
	firstPointList = creatTrajPoint(myGraph, firstPath, initialVertex);
	secPointList   = creatTrajPoint(myGraph, secPath, initialVertex);
	Point p[firstPointList.size()];
	Point q[secPointList.size()];
	for (unsigned int i = 0; i < firstPointList.size(); i++)
	{
		p[i].x = firstPointList[i].x;
		p[i].y = firstPointList[i].y;
	}
	for (unsigned int i = 0; i < secPointList.size(); i++)
	{
		q[i].x = secPointList[i].x;
		q[i].y = secPointList[i].y;
	}
	return dfdDist(firstPointList.size(), secPointList.size(), p, q);
}