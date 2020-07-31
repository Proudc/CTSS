#include <stdio.h>
#include <stdbool.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>
#include <ctime>
#include <limits>

#include "../../include/ContinuousQuery/Base.h"
#include "../../include/ContinuousQuery/Common.h"
#include "../../include/ContinuousQuery/Discrete.h"
#include "../../include/ContinuousQuery/Frechet.h"
#include "../../include/ContinuousQuery/Pruneline.h"

using namespace std;

namespace
{

std::vector<Sequence> setIncreDecreInterval(double calHelpArray[], TraversingPath* path)
{
	int  startPos = path->minPos;
	int  stopPos;
	int  flag = 0;
	bool seqFlag;
	std::vector<Sequence> myInterval;
	if (calHelpArray[path->minPos] < calHelpArray[(path->minPos) + 1])
	{
		seqFlag = true;
	}
	else
	{
		seqFlag = false;
	}
	for (unsigned int i = (path->minPos) + 1; i < (path->maxPos);)
	{
		if (abs(calHelpArray[i] - (-1.0)) < 0.00000001)
		{
			Sequence* temSequence = new Sequence;
			temSequence->startPos = startPos;
			temSequence->stopPos  = i - 1;
			temSequence->seqFlag  = seqFlag;
			myInterval.push_back(*temSequence);
			delete temSequence;
			while ((i <= path->maxPos) && (abs(calHelpArray[i] - (-1.0)) < 0.0000001))
			{
				i++;
			}
			if (i > (path->maxPos))
			{
				flag = 1;
				break;
			}
			startPos = i;
			if (calHelpArray[i] < calHelpArray[i + 1])
			{
				seqFlag = true;
			}
			else
			{
				seqFlag = false;
			}
		}
		else
		{
			if (abs(calHelpArray[i + 1] - (-1.0)) < 0.000000001)
			{
				i++;
				continue;
			}
			bool temSeqFlag;
			if (calHelpArray[i] == calHelpArray[i + 1])
			{
				i++;
				continue;
			}
			else if (calHelpArray[i] < calHelpArray[i + 1])
			{
				temSeqFlag = true;
			}
			else
			{
				temSeqFlag = false;
			}
			if (seqFlag != temSeqFlag)
			{
				Sequence* temSequence = new Sequence;
				temSequence->startPos = startPos;
				temSequence->stopPos  = i;
				temSequence->seqFlag  = seqFlag;
				myInterval.push_back(*temSequence);
				delete temSequence;
				startPos = i + 1;
				seqFlag = temSeqFlag;
			}
			i++;
		}
	}
	if (flag == 0)
	{
		Sequence* temSequence = new Sequence;
		temSequence->startPos = startPos;
		temSequence->stopPos  = path->maxPos;
		temSequence->seqFlag  = seqFlag;
		myInterval.push_back(*temSequence);
		delete temSequence;
	}
	return myInterval;
}

/**
 * Calculate lowerbound in Euclidean space
 * Can only be used in European space
 * The reason is that once it encounters a lowerbound that meets the delta, it returns
 * @param myGraph pointer to a graph
 * @param path pointer to a TraversingPath
 * @param record information to be recorded during calculation
 * @param referencePath pointer to a ReferencePath
 * @return A lowerbound that meets the delta
*/
double eucLowerBound(Graph* myGraph, TraversingPath* path, Record* record, ReferencePath* referencePath)
{
	printf("Start calculating LowerBound...\n");
	record->numDFDCal       = 0;
	record->numTemDiscrete  = 0;
	record->locationOfRefer = -1;

	if ((myGraph->vertexsVector[path->finalVertexID].locationFlag == false) || (path->maxPos < path->minPos))
	{
		return -1.0;
	}

	std::vector<Edge>  finalPath;
	std::vector<Point> temPointList;
	for (unsigned int i = 0; i < path->edges.size(); i++)
	{
		finalPath.push_back(myGraph->edgesVector[path->edges[i]]);
	}
	temPointList = creatTrajPoint(myGraph, finalPath, referencePath->initialVertex);
	record->numTemDiscrete = temPointList.size();

	double calHelpArray[10000];
	for (unsigned int i = path->minPos; i <= path->maxPos; i++)
	{
		double x1 = myGraph->vertexsVector[path->finalVertexID].longitude;
		double y1 = myGraph->vertexsVector[path->finalVertexID].latitude;
		double x2 = referencePath->pointOfRefer[i].x;
		double y2 = referencePath->pointOfRefer[i].y;
		double eucDist = euc(x1, y1, x2, y2);
		if (eucDist <= (myGraph->delta))
		{
			calHelpArray[i] = eucDist;
		}
		else
		{
			calHelpArray[i] = -1.0;
		}
	}

	std::vector<Sequence> myInterval = setIncreDecreInterval(calHelpArray, path);

	double temDist = std::numeric_limits<double>::max();
	std::vector<std::vector<double>> eucDistance(1000, std::vector<double>(1000, -1));
	std::vector<std::vector<double>> dfdDistance(1000, std::vector<double>(1000, -1));
	for (unsigned int i = 0; i < myInterval.size(); i++)
	{
		if (myInterval[i].seqFlag == false)
		{
			for (unsigned int k = 0; k < temPointList.size(); k++)
			{
				for (unsigned int j = 0; j <= myInterval[i].stopPos; j++)
				{
					double x1 = temPointList[k].x;
					double y1 = temPointList[k].y;
					double x2 = referencePath->pointOfRefer[j].x;
					double y2 = referencePath->pointOfRefer[j].y;
					double eucDist = euc(x1, y1, x2, y2);
					eucDistance[k][j] = eucDist;
					if ((k == 0) && (j == 0))
					{
						dfdDistance[k][j] = eucDistance[k][j];
					}
					else if ((k == 0) && (j != 0))
					{
						dfdDistance[k][j] = (std::max)(dfdDistance[k][j - 1], eucDistance[k][j]);
					}
					else if ((k != 0) && (j == 0))
					{
						dfdDistance[k][j] = (std::max)(dfdDistance[k - 1][j], eucDistance[k][j]);
					}
					else
					{
						dfdDistance[k][j] = (std::max)(
							(std::min)((std::min)(dfdDistance[k - 1][j], dfdDistance[k][j - 1]), dfdDistance[k - 1][j - 1]), eucDistance[k][j]);
					}
				}
			}
			if (dfdDistance[temPointList.size() - 1][myInterval[i].stopPos] < temDist)
			{
				record->eucConQueryPos = myInterval[i].stopPos;
			}

			if (dfdDistance[temPointList.size() - 1][myInterval[i].stopPos] < temDist)
			{
				temDist = dfdDistance[temPointList.size() - 1][myInterval[i].stopPos];
			}
			record->locationOfRefer = myInterval[i].stopPos;
			record->numDFDCal += 1;
			if (temDist <= (myGraph->delta))
			{
				return temDist;
			}
		}
		else
		{
			for (unsigned int k = 0; k <= temPointList.size(); k++)
			{
				for (unsigned int j = 0; j < myInterval[i].startPos; j++)
				{
					double x1 = temPointList[k].x;
					double y1 = temPointList[k].y;
					double x2 = referencePath->pointOfRefer[j].x;
					double y2 = referencePath->pointOfRefer[j].y;
					double eucDist = euc(x1, y1, x2, y2);
					eucDistance[k][j] = eucDist;
					if ((k == 0) && (j == 0))
					{
						dfdDistance[k][j] = eucDistance[k][j];
					}
					else if ((k == 0) && (j != 0))
					{
						dfdDistance[k][j] = (std::max)(dfdDistance[k][j - 1], eucDistance[k][j]);
					}
					else if ((k != 0) && (j == 0))
					{
						dfdDistance[k][j] = (std::max)(dfdDistance[k - 1][j], eucDistance[k][j]);
					}
					else
					{
						dfdDistance[k][j] = (std::max)(
							(std::min)((std::min)(dfdDistance[k - 1][j], dfdDistance[k][j - 1]), dfdDistance[k - 1][j - 1]), eucDistance[k][j]);
					}
				}
			}
			record->numDFDCal += 1;
			for (unsigned int j = myInterval[i].startPos; j <= myInterval[i].stopPos; j++)
			{
				for (unsigned int k = 0; k < temPointList.size(); k++)
				{
					double x1 = temPointList[k].x;
					double y1 = temPointList[k].y;
					double x2 = referencePath->pointOfRefer[j].x;
					double y2 = referencePath->pointOfRefer[j].y;
					double eucDist = euc(x1, y1, x2, y2);
					eucDistance[k][j] = eucDist;
					if ((k == 0) && (j == 0))
					{
						dfdDistance[k][j] = eucDistance[k][j];
					}
					else if ((k == 0) && (j != 0))
					{
						dfdDistance[k][j] = (std::max)(dfdDistance[k][j - 1], eucDistance[k][j]);
					}
					else if ((k != 0) && (j == 0))
					{
						dfdDistance[k][j] = (std::max)(dfdDistance[k - 1][j], eucDistance[k][j]);
					}
					else
					{
						dfdDistance[k][j] = (std::max)(
							(std::min)((std::min)(dfdDistance[k - 1][j], dfdDistance[k][j - 1]), dfdDistance[k - 1][j - 1]), eucDistance[k][j]);
					}
				}
				if (dfdDistance[temPointList.size() - 1][j] < temDist)
				{
					record->eucConQueryPos = j;
				}
				if (dfdDistance[temPointList.size() - 1][j] < temDist)
				{
					temDist = dfdDistance[temPointList.size() - 1][j];
				}
				record->locationOfRefer = j;
				if (temDist <= (myGraph->delta))
				{
					return temDist;
				}
				if (j != myInterval[i].stopPos)
				{
					double x1 = temPointList[temPointList.size() - 1].x;
					double y1 = temPointList[temPointList.size() - 1].y;
					double x2 = referencePath->pointOfRefer[j + 1].x;
					double y2 = referencePath->pointOfRefer[j + 1].y;
					if (euc(x1, y1, x2, y2) >= dfdDistance[temPointList.size() - 1][j])
					{
						break;
					}
				}
			}
		}
	}
	return temDist;
}

bool calculateLowerBound(Graph* myGraph, TraversingPath* path, Record* record, ReferencePath* referencePath)
{
	printf("Start calculating lower bound...\n");
	record->numDFDCal = 0;
	record->numTemDiscrete = 0;
	record->locationOfRefer = -1;
	if (path->maxPos < path->minPos)
	{
		return -1.0;
	}
	else
	{
		std::vector<Edge>  finalPath;
		std::vector<Point> temPointList;
		for (unsigned int i = 0; i < path->edges.size(); i++)
		{
			finalPath.push_back(myGraph->edgesVector[path->edges[i]]);
		}
		temPointList = creatTrajPoint(myGraph, finalPath, referencePath->initialVertex);
		Point p[(temPointList.size())];
		for (unsigned int i = 0; i < temPointList.size(); i++)
		{
			p[i].x = temPointList[i].x;
			p[i].y = temPointList[i].y;
		}
		printf("123456789\n");
		double calHelpArray[10000];
		for (unsigned int i = path->minPos; i <= path->maxPos; i++)
		{
			double x1 = myGraph->vertexsVector[path->finalVertexID].longitude;
			double y1 = myGraph->vertexsVector[path->finalVertexID].latitude;
			double x2 = referencePath->pointOfRefer[i].x;
			double y2 = referencePath->pointOfRefer[i].y;
			double eucDist = euc(x1, y1, x2, y2);
			if (eucDist <= (myGraph->delta))
			{
				calHelpArray[i] = eucDist;
			}
			else
			{
				calHelpArray[i] = -1.0;
			}
		}
		std::vector<Sequence> myInterval = setIncreDecreInterval(calHelpArray, path);
		printf("987654321\n");
		double   temDist = std::numeric_limits<double>::max();
		std::vector<std::vector<double>> eucDistance(1000, std::vector<double>(1000, -1));
		std::vector<std::vector<double>> dfdDistance(1000, std::vector<double>(1000, -1));
		for (unsigned int i = 0; i < myInterval.size(); i++)
		{
			printf("555\n");
			if (myInterval[i].seqFlag == false)
			{
				printf("111\n");
				double xDist = dfdDist(temPointList.size(), myInterval[i].stopPos + 1, p, referencePath->pointOfRefer);
				temDist = xDist < temDist ? xDist : temDist;
				printf("temDist is: %lf\n", temDist);
				printf("222\n");
			}
			else
			{
				printf("666\n");
				for (unsigned int k = 0; k < temPointList.size(); k++)
				{
					for (unsigned int j = 0; j < myInterval[i].startPos; j++)
					{
						double x1 = p[k].x;
						double y1 = p[k].y;
						double x2 = referencePath->pointOfRefer[j].x;
						double y2 = referencePath->pointOfRefer[j].y;
						double eucDist = euc(x1, y1, x2, y2);
						eucDistance[k][j] = eucDist;
						if ((k == 0) && (j == 0))
						{
							dfdDistance[k][j] = eucDistance[k][j];
						}
						else if ((k == 0) && (j != 0))
						{
							dfdDistance[k][j] = (std::max)(dfdDistance[k][j - 1], eucDistance[k][j]);
						}
						else if ((k != 0) && (j == 0))
						{
							dfdDistance[k][j] = (std::max)(dfdDistance[k - 1][j], eucDistance[k][j]);
						}
						else
						{
							dfdDistance[k][j] = (std::max)(
								(std::min)((std::min)(dfdDistance[k - 1][j], dfdDistance[k][j - 1]), dfdDistance[k - 1][j - 1]), eucDistance[k][j]);
						}
					}
				}
				printf("1365\n");
				for (unsigned int j = myInterval[i].startPos; j <= myInterval[i].stopPos; j++)
				{
					for (unsigned int k = 0; k < temPointList.size(); k++)
					{
						double x1 = p[k].x;
						double y1 = p[k].y;
						double x2 = referencePath->pointOfRefer[j].x;
						double y2 = referencePath->pointOfRefer[j].y;
						double eucDist = euc(x1, y1, x2, y2);
						eucDistance[k][j] = eucDist;
					}
					if (dfdDistance[temPointList.size() - 1][j] < temDist)
					{
						temDist = dfdDistance[temPointList.size() - 1][j];
					}
					if (j != myInterval[i].stopPos)
					{
						double x1 = p[temPointList.size() - 1].x;
						double y1 = p[temPointList.size() - 1].y;
						double x2 = referencePath->pointOfRefer[j + 1].x;
						double y2 = referencePath->pointOfRefer[j + 1].y;
						if (euc(x1, y1, x2, y2) >= dfdDistance[temPointList.size() - 1][j])
						{
							break;
						}
					}
				}
				printf("78978\n");
			}

		}
		printf("7894564\n");
		printf("654\n");
		printf("final temDist is:%lf\n", temDist);
		return temDist;
	}
}

bool BFS(Graph* myGraph, ReferencePath* referencePath, std::vector<Edge> currentPath, int currentVertex)
{
	printf("Start BFS search...\n");
	bool bfsFlag[(myGraph->vertexPos)];
	for (unsigned int i = 0; i < myGraph->vertexPos; i++)
	{
		bfsFlag[i] = false;
	}
	Vertex* firstVertex = new Vertex;
	firstVertex->vertexID = currentVertex;
	std::queue<Vertex> queue;
	queue.push(*firstVertex);
	while (!queue.empty())
	{
		Vertex temVertex = queue.front();
		queue.pop();
		bfsFlag[temVertex.vertexID] = true;
		if (myGraph->vertexsVector[temVertex.vertexID].referFlag == false)
		{
			for (unsigned int i = 0; i < myGraph->vertexsVector[temVertex.vertexID].adjVertexID.size(); i++)
			{
				int nextVertexID = myGraph->vertexsVector[temVertex.vertexID].adjVertexID[i];
				if ((bfsFlag[nextVertexID] == false) && (myGraph->vertexsVector[nextVertexID].currFlag == false) && (myGraph->vertexsVector[nextVertexID].locationFlag == true))
				{
					Vertex* newNode = new Vertex;
					newNode = &(myGraph->vertexsVector[nextVertexID]);
					newNode->parentVertexID = temVertex.vertexID;
					bfsFlag[nextVertexID] = true;
					queue.push(*newNode);
				}
			}
		}
		else
		{
			std::vector<Edge> temPath;
			Vertex* newNode;
			int temID = temVertex.vertexID;
			while (temID != currentVertex)
			{
				temPath.push_back(findEdge(myGraph, myGraph->vertexsVector[temID], myGraph->vertexsVector[myGraph->vertexsVector[temID].parentVertexID]));
				temID = myGraph->vertexsVector[temID].parentVertexID;
			}
			std::vector<Edge> finalPath;
			for (unsigned int i = 0; i < currentPath.size(); i++)
			{
				finalPath.push_back(currentPath[i]);
			}
			for (int i = (temPath.size() - 1); i >= 0; i--)
			{
				finalPath.push_back(temPath[i]);
			}
			int pathPos;
			int finalVertexID = temVertex.vertexID;
			for (int i = referencePath->edges.size() - 1; i >= 0; i--)
			{
				if ((referencePath->edges[i].firstVertexID == finalVertexID) || (referencePath->edges[i].secVertexID == finalVertexID))
				{
					pathPos = i;
					break;
				}
			}
			for (unsigned int i = pathPos; i < referencePath->edges.size(); i++)
			{
				finalPath.push_back(referencePath->edges[i]);
			}
			std::vector<Point> temPointList;
			temPointList = creatTrajPoint(myGraph, finalPath, referencePath->initialVertex);
			Point p[temPointList.size()];
			for (unsigned int i = 0; i < temPointList.size(); i++)
			{
				p[i].x = temPointList[i].x;
				p[i].y = temPointList[i].y;
			}
			double temDist = dfdDist(temPointList.size(), referencePath->numOfReferPoint, p, referencePath->pointOfRefer);
			if (temDist <= (myGraph->delta))
			{
				return true;
			}
			else
			{
				return false;
			}
		}
	}
	return false;
}

}

bool eucSimpleQueryPath(Graph* myGraph, ReferencePath* referencePath, std::vector<Edge> currentPath, Record* temRecord, int currentVertex)
{
	printf("Start eucSimpleQueryPath...\n");
	TraversingPath* path = new TraversingPath;
	initializeFirstPath(myGraph, path, currentVertex, currentPath);
	if (path->finalVertexID != referencePath->finalVertex)
	{
		double lb = eucLowerBound(myGraph, path, temRecord, referencePath);
		if ((lb != -1) && (lb < (myGraph->delta)))
		{
			delete path;
			return true;
		}
		else
		{
			delete path;
			return false;
		}
	}
	else
	{
		std::vector<Edge> finalPath;
		for (unsigned int i = 0; i < path->edges.size(); i++)
		{
			finalPath.push_back(myGraph->edgesVector[path->edges[i]]);
		}
		double temDist = DFD(myGraph, finalPath, referencePath);
		if (temDist <= (myGraph->delta))
		{
			delete path;
			return true;
		}
		else
		{
			delete path;
			return false;
		}
	}
}

bool roadSimpleQueryPath(Graph* myGraph, ReferencePath* referencePath, std::vector<Edge> currentPath, 
	std::vector<Edge>& lastPath, Record* temRecord, int currentVertex, int* timeOutFlag, double* pathDistance)
{
	if (!BFS(myGraph, referencePath, currentPath, currentVertex))
	{
		printf("BFS search did not get results...\n");
		clock_t startTime, stopTime;
		startTime = clock();
		priority_queue<TraversingPath> priorityQueue;
		TraversingPath* firstPath = new TraversingPath;
		initializeFirstPath(myGraph, firstPath, currentVertex, currentPath);
		priorityQueue.push(*firstPath);
		temRecord->numOfOutPriQueue = 0;
		while (!priorityQueue.empty())																				  
		{
			TraversingPath* temPath;
			stopTime = clock();
			if (((double)(stopTime - startTime) / CLOCKS_PER_SEC) > 10)
			{
				(*timeOutFlag) = 1;
				delete firstPath;
				return false;
			}

			(*temPath) = (priorityQueue.top());
			priorityQueue.pop();
			temRecord->numOfOutPriQueue += 1;
			
			if (temPath->finalVertexID != referencePath->finalVertex)
			{
				for (unsigned int i = 0; i < (myGraph->vertexsVector[temPath->finalVertexID].adjVertexID.size()); i++)
				{
					int nextVertexID = myGraph->vertexsVector[temPath->finalVertexID].adjVertexID[i];
					if ((myGraph->vertexsVector[nextVertexID].locationFlag == true) && (checkIfAccess(myGraph, temPath, nextVertexID)))
					{
						TraversingPath* newPath = new TraversingPath;
						handleNewPath(myGraph, temPath, newPath, nextVertexID);

						if ((myGraph->vertexsVector[newPath->finalVertexID].referFlag == true))
						{
							double temDist = dfdDistBetweenTwoPaths(myGraph, newPath, referencePath, lastPath, newPath->finalVertexID);
							if (temDist <= (myGraph->delta))
							{
								delete firstPath;
								delete newPath;
								(*pathDistance) = temDist;
								return true;
							}
							else if ((newPath->finalVertexID == referencePath->finalVertex) && (referencePath->referDFDFlag == true))
							{
								delete newPath;
								continue;
							}
						}
						else
						{
							double lb = calculateLowerBound(myGraph, newPath, temRecord, referencePath);
							if ((lb != -1) && (lb < (myGraph->delta)))
							{
								newPath->eucDeltaOfTrue = lb;
								newPath->eucDelta = (100000 * lb + myGraph->vertexsVector[newPath->finalVertexID].traversalOrderWeights);
								priorityQueue.push(*newPath);
								delete(newPath);
							}
						}
					}
				}
			}
			else
			{
				std::vector<Edge> finalPath;
				for (unsigned int i = 0; i < temPath->edges.size(); i++)
				{
					finalPath.push_back(myGraph->edgesVector[temPath->edges[i]]);
				}

				double temDist = DFD(myGraph, finalPath, referencePath);
				if (temDist <= (myGraph->delta))
				{
					delete firstPath;
					lastPath = finalPath;
					(*pathDistance) = temDist;
					return true;
				}
			}
		}
		delete firstPath;
		return false;
	}
	else
	{
		return true;
	}

}

bool roadSimleQueryPathWithLoop(Graph* myGraph, ReferencePath* referencePath, std::vector<Edge> currentPath, 
	std::vector<Edge>& lastPath, Record* temRecord, int currentVertex, int* timeOutFlag, double* pathDistance)
{
	clock_t startTime, stopTime;
	startTime = clock();
	priority_queue<TraversingPath> priorityQueue;
	TraversingPath* firstPath = new TraversingPath;
	initializeFirstPath(myGraph, firstPath, currentVertex, currentPath);
	priorityQueue.push(*firstPath);
	temRecord->numOfOutPriQueue = 0;
	while (!priorityQueue.empty())
	{
		TraversingPath* temPath;
		stopTime = clock();
		if (((double)(stopTime - startTime) / CLOCKS_PER_SEC) > 10)
		{
			(*timeOutFlag) = 1;
			delete firstPath;
			return false;
		}

		(*temPath) = (priorityQueue.top());
		priorityQueue.pop();
		temRecord->numOfOutPriQueue += 1;

		if (temPath->edges.size() == 4990)
		{
			continue;
		}

		if (temPath->finalVertexID != referencePath->finalVertex)
		{
			for (unsigned int i = 0; i < (myGraph->vertexsVector[temPath->finalVertexID].adjVertexID.size()); i++)
			{
				int nextVertexID = myGraph->vertexsVector[temPath->finalVertexID].adjVertexID[i];
				if (checkIfAccess(myGraph, temPath, nextVertexID))
				{

					double maxEuc = 0;
					double beginEuc;
					double endEuc;
					for (unsigned int j = myGraph->vertexsVector[nextVertexID].minPos; j <= myGraph->vertexsVector[nextVertexID].maxPos; j++)
					{
						double x1 = myGraph->vertexsVector[nextVertexID].longitude;
						double y1 = myGraph->vertexsVector[nextVertexID].latitude;
						double x2 = referencePath->pointOfRefer[j].x;
						double y2 = referencePath->pointOfRefer[j].y;
						double eucDist = euc(x1, y1, x2, y2);
						if (j == myGraph->vertexsVector[nextVertexID].minPos)
						{
							beginEuc = eucDist;
						}
						if (j == myGraph->vertexsVector[nextVertexID].maxPos)
						{
							endEuc = eucDist;
						}
						if (eucDist <= (myGraph->delta))
						{
							maxEuc = (std::max)(eucDist, maxEuc);
						}
					}
					if ((abs(maxEuc - beginEuc) < 0.000001) || (abs(maxEuc - endEuc) < 0.000001))
					{
						continue;
					}

					if ((myGraph->vertexsVector[nextVertexID].locationFlag == true))
					{
						TraversingPath* newPath = new TraversingPath;
						handleNewPath(myGraph, temPath, newPath, nextVertexID);
						if ((myGraph->vertexsVector[newPath->finalVertexID].referFlag == true))
						{
							double temDist = dfdDistBetweenTwoPaths(myGraph, newPath, referencePath, lastPath, newPath->finalVertexID);
							if (temDist <= (myGraph->delta))
							{
								delete firstPath;
								delete newPath;
								(*pathDistance) = temDist;
								return true;
							}
							else if ((newPath->finalVertexID == referencePath->finalVertex) && (referencePath->referDFDFlag == true))
							{
								delete newPath;
								continue;
							}
						}
						else
						{
							double lb = calculateLowerBound(myGraph, newPath, temRecord, referencePath);
							if ((lb != -1) && (lb < (myGraph->delta)))
							{
								newPath->eucDeltaOfTrue = lb;
								newPath->eucDelta = (100000 * lb + myGraph->vertexsVector[newPath->finalVertexID].traversalOrderWeights);
								priorityQueue.push(*newPath);
								delete newPath;
							}
						}

					}
				}
			}
		}
		else
		{
			std::vector<Edge> finalPath;
			for (unsigned int i = 0; i < temPath->edges.size(); i++)
			{
				finalPath.push_back(myGraph->edgesVector[temPath->edges[i]]);
			}
			double temDist = DFD(myGraph, finalPath, referencePath);
			
			if (temDist <= (myGraph->delta))
			{
				delete firstPath;
				lastPath = finalPath;
				(*pathDistance) = temDist;
				return true;
			}
		}
	}
	return false;
}

void maxmalOverlapPath(Graph* myGraph, std::vector<Edge>& localPath, std::vector<Edge>& lastPath, int firstVertexID, int secVertexID, int* endVertexID)
{
	bool bfsFlag[(myGraph->vertexPos)];
	bool lastPathFlag[(myGraph->vertexPos)];
	for (unsigned int i = 0; i < myGraph->vertexPos; i++)
	{
		bfsFlag[i] = false;
		lastPathFlag[i] = false;
	}
	for (unsigned int i = 0; i < lastPath.size(); i++)
	{
		Edge temEdge = lastPath[i];
		if (temEdge.firstVertexID == firstVertexID || temEdge.secVertexID == firstVertexID)
		{
			if (temEdge.firstVertexID == firstVertexID)
			{
				bfsFlag[temEdge.firstVertexID] = true;
			}
			else
			{
				bfsFlag[temEdge.secVertexID] = true;
			}
			break;
		}
		else
		{
			bfsFlag[temEdge.firstVertexID] = true;
			bfsFlag[temEdge.secVertexID]   = true;
		}
	}
	for (unsigned int i = 0; i < lastPath.size(); i++)
	{
		lastPathFlag[lastPath[i].firstVertexID] = true;
		lastPathFlag[lastPath[i].secVertexID]   = true;
	}
	Vertex* initVertex = new Vertex;
	initVertex->vertexID = secVertexID;
	std::queue<Vertex> queue;
	queue.push(*initVertex);
	while (!queue.empty())
	{
		Vertex temVertex = queue.front();
		queue.pop();
		bfsFlag[temVertex.vertexID] = true;
		if (lastPathFlag[temVertex.vertexID] == false)
		{
			for (unsigned int i = 0; i < myGraph->vertexsVector[temVertex.vertexID].adjVertexID.size(); i++)
			{
				int nextVertexID = myGraph->vertexsVector[temVertex.vertexID].adjVertexID[i];
				if (bfsFlag[nextVertexID] == false && myGraph->vertexsVector[nextVertexID].locationFlag == true)
				{
					bfsFlag[nextVertexID] = true;
					Vertex* newVertex = new Vertex;
					newVertex = &myGraph->vertexsVector[nextVertexID];
					newVertex->parentVertexID = temVertex.vertexID;
					queue.push(*newVertex);
				}
			}
		}
		else
		{
			(*endVertexID) = temVertex.vertexID;
			std::vector<Edge> temPath;
			Vertex newVertex;
			int temID = temVertex.vertexID;
			while (myGraph->vertexsVector[temID].vertexID != secVertexID)
			{
				int temParentID = myGraph->vertexsVector[temID].parentVertexID;
				temPath.push_back(findEdge(myGraph, myGraph->vertexsVector[temID], myGraph->vertexsVector[temParentID]));
				temID = temParentID;
			}
			localPath.push_back(findEdge(myGraph, myGraph->vertexsVector[firstVertexID], myGraph->vertexsVector[secVertexID]));
			for (int i = temPath.size() - 1; i >= 0; i--)
			{
				localPath.push_back(temPath[i]);
			}
		}
	}
}
