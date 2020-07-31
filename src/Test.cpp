#include <stdio.h>
#include <stdbool.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <map>
#include <queue>
#include <ctime>
#include <limits>
#include <iostream>
#include <fstream>

#include "../include/ContinuousQuery/Base.h"
#include "../include/ContinuousQuery/Common.h"
#include "../include/ContinuousQuery/Construct.h"
#include "../include/ContinuousQuery/Discrete.h"
#include "../include/ContinuousQuery/Frechet.h"
#include "../include/ContinuousQuery/Pruneline.h"

#pragma GCC optimize(2)

using namespace std;
void eucSimpleQuery(Graph* myGraph, string inputFolder, string outputFolder, const int startCount, const int stopCount);
void eucContinuousQuery(Graph* myGraph, string inputFolder, string outputFolder, const int startCount, const int stopCount);
void roadSimpleQuery(Graph* myGraph, string inputFolder, string outputFolder, const int startCount, const int stopCount);
void roadContinuousQuery(Graph* myGraph, string inputFolder, string outputFolder, const int startCount, const int stopCount);


/**
 * Determine query based on argv[]
 * argv[1] is the input folder path
 * argv[2] is the output folder path
 * argv[3] is the starting position of the query
 * argv[4] is the stoping position of the query
 * argv[5] is the delta
 * argv[6] is the query mode. 1:Single point query in Euclidean space
 *                            2:Continuous query in Euclidean space
 *                            3:Single point query in Road network space
 *                            4:Continuous query in Road network space
 * argv[7] is the section length
*/
int main(int argc, char* argv[])
{
	string inputFolder(argv[1]);
	string outputFolder(argv[2]);
	int    startCount    = atoi(argv[3]);
	int    stopCount     = atoi(argv[4]);
	int    mode          = atoi(argv[6]);
	double delta         = atof(argv[5]);
	double sectionLength = atof(argv[7]);

	string nodePath = inputFolder + "/nodeOSM.txt";
	string edgePath = inputFolder + "/edgeOSM.txt";
	Graph* myGraph  = new Graph;
	initializeGraph(myGraph, nodePath, edgePath, delta, sectionLength);

	if (mode == 1)
	{
		eucSimpleQuery(myGraph, inputFolder, outputFolder, startCount, stopCount);
	}
	else if (mode == 2)
	{
		eucContinuousQuery(myGraph, inputFolder, outputFolder, startCount, stopCount);
	}
	else if (mode == 3)
	{
		roadSimpleQuery(myGraph, inputFolder, outputFolder, startCount, stopCount);
	}
	else if (mode == 4)
	{
		roadContinuousQuery(myGraph, inputFolder, outputFolder, startCount, stopCount);
	}
	else
	{
		printf("Sorry, You should enter the correct query mode!\n");
	}
	return 0;
}

/**
 * Single point query in Euclidean space
 * @param myGraph graph for query
 * @param inputFloder folder path when reading files
 * @param outputFolder folder path when writing files
 * @param startCount the start position of the query
 * @param stopCount the stop position of the querys
*/
void eucSimpleQuery(Graph* myGraph, string inputFolder, string outputFolder, const int startCount, const int stopCount)
{
	int count = startCount;
	while (count <= stopCount)
	{
		printf("Start %dth query...\n", count);
		string strCount = changeIntToStr(count);
		ReferencePath* referencePath = new ReferencePath;
		CompletePath*  completePath  = new CompletePath;
		setQueryEnv(myGraph, referencePath, completePath, inputFolder, strCount);
		
		int    currentVertex = completePath->initialVertex;
		int    recordPos = 0;
		Record record[completePath->edges.size()];
		double currPathLength = 0.0;
		for (unsigned int i = 1; i <= completePath->edges.size(); i++)
		{
			currPathLength += completePath->edges[i - 1].realLength;
			if (completePath->edges[i - 1].firstVertexID == currentVertex)
			{
				currentVertex = completePath->edges[i - 1].secVertexID;
			}
			else
			{
				currentVertex = completePath->edges[i - 1].firstVertexID;
			}
			
			vector<Edge> currentPath;
			setCurrPath(myGraph, &currentPath, completePath, i);
			Record* temRecord = new Record;
			
			clock_t startTime = clock();
			bool queryResultFlag = eucSimpleQueryPath(myGraph, referencePath, currentPath, temRecord, currentVertex);
			clock_t stopTime = clock();

			temRecord->ratioNumEdge = ((double)(i)) / completePath->edges.size();
			temRecord->runTime = (double)(stopTime - startTime) / CLOCKS_PER_SEC;
			temRecord->comPathSize = completePath->edges.size();
			temRecord->numOfOutPriQueue = 0;
			temRecord->referPathLength = referencePath->referenceLength;
			temRecord->ratioPathLength = currPathLength / (completePath->completeLength);
			temRecord->currPathLength = currPathLength;
			temRecord->comPathLength = completePath->completeLength;
			temRecord->numReferDiscrete = referencePath->numOfReferPoint;
			temRecord->referPathSize = referencePath->edges.size();
			record[recordPos++] = (*temRecord);

			if (queryResultFlag == false)
			{
				break;
			}
		}
		string writeFilePath = outputFolder + "/" + strCount + "partial.txt";
		writeSingleRecordToFile(record, recordPos, writeFilePath);
		
		delete referencePath;
		delete completePath;
		count++;
	}
}

/**
 * Continuous query in Euclidean space
 * @param myGraph graph for query
 * @param inputFloder folder path when reading files
 * @param outputFolder folder path when writing files
 * @param startCount the start position of the query
 * @param stopCount the stop position of the querys
*/
void eucContinuousQuery(Graph* myGraph, string inputFolder, string outputFolder, const int startCount, const int stopCount)
{
	int count = startCount;
	while (count <= stopCount)
	{
		printf("Start %dth query...\n", count);
		string strCount = changeIntToStr(count);
		ReferencePath* referencePath = new ReferencePath;
		CompletePath*  completePath  = new CompletePath;
		setQueryEnv(myGraph, referencePath, completePath, inputFolder, strCount);
		
		int    currentVertex = completePath->initialVertex;
		int    recordPos = 0;
		Record record[completePath->edges.size()];
		double currPathLength = 0.0;
		int    temConQueryPos = 0;
		clock_t startTime = clock();
		for (unsigned int i = 1; i < completePath->edges.size(); i++)
		{
			currPathLength += completePath->edges[i - 1].realLength;
			if (completePath->edges[i - 1].firstVertexID == currentVertex)
			{
				currentVertex = completePath->edges[i - 1].secVertexID;
			}
			else
			{
				currentVertex = completePath->edges[i - 1].firstVertexID;
			}

			vector<Edge> currentPath;
			setCurrPath(myGraph, &currentPath, completePath, i);
			
			Record* temRecord = new Record;
			temRecord->ratioNumEdge     = ((double)(i)) / completePath->edges.size();
			temRecord->runTime          = 0;
			temRecord->comPathSize      = completePath->edges.size();
			temRecord->numOfOutPriQueue = 0;
			temRecord->referPathLength  = referencePath->referenceLength;
			temRecord->ratioPathLength  = currPathLength / (completePath->completeLength);
			temRecord->currPathLength   = currPathLength;
			temRecord->comPathLength    = completePath->completeLength;
			record[recordPos++] = (*temRecord);

			
			double x1 = myGraph->vertexsVector[currentVertex].longitude;
			double y1 = myGraph->vertexsVector[currentVertex].latitude;
			double x2 = referencePath->pointOfRefer[temConQueryPos].x;
			double y2 = referencePath->pointOfRefer[temConQueryPos].y;
			if (euc(x1, y1, x2, y2) <= (myGraph->delta))
			{
				continue;
			}
			else
			{
				temRecord->eucConQueryPos = temConQueryPos;
				bool queryResultFlag = eucSimpleQueryPath(myGraph, referencePath, currentPath, temRecord, currentVertex);
				temConQueryPos = temRecord->eucConQueryPos;
				if (queryResultFlag == false)
				{
					break;
				}
			}

		}
		clock_t stopTime = clock();
		string writeFilePath = outputFolder + "/" + strCount + "eucContinuousQuery.txt";
		writeContinuousRecordToFile(record, recordPos, writeFilePath, (double)(stopTime - startTime) / CLOCKS_PER_SEC);
		
		delete referencePath;
		delete completePath;
		count++;
	}
}

/**
 * Single point query in Road network space
 * @param myGraph graph for query
 * @param inputFloder folder path when reading files
 * @param outputFolder folder path when writing files
 * @param startCount the start position of the query
 * @param stopCount the stop position of the querys
*/
void roadSimpleQuery(Graph* myGraph, string inputFolder, string outputFolder, const int startCount, const int stopCount)
{
	int count = startCount;
	while (count <= stopCount)
	{
		printf("Start %dth query...\n", count);
		string strCount = changeIntToStr(count);
		ReferencePath* referencePath = new ReferencePath;
		CompletePath*  completePath  = new CompletePath;
		setQueryEnv(myGraph, referencePath, completePath, inputFolder, strCount);
		
		int    currentVertex = completePath->initialVertex;
		int    recordPos = 0;
		Record record[completePath->edges.size()];
		double currPathLength = 0.0;
		double temDistance;
		vector<Edge> temPath;
		int    timeFlag = 0;
		for (unsigned int i = 1; i < completePath->edges.size(); i++)
		{
			currPathLength += completePath->edges[i - 1].realLength;
			if (completePath->edges[i - 1].firstVertexID == currentVertex)
			{
				currentVertex = completePath->edges[i - 1].secVertexID;
			}
			else
			{
				currentVertex = completePath->edges[i - 1].firstVertexID;
			}
			
			vector<Edge> currentPath;
			setCurrPath(myGraph, &currentPath, completePath, i);
			
			Record* temRecord = new Record;
			temRecord->ratioNumEdge     = ((double)(i)) / completePath->edges.size();
			temRecord->comPathSize      = completePath->edges.size();
			temRecord->referPathLength  = referencePath->referenceLength;
			temRecord->ratioPathLength  = currPathLength / (completePath->completeLength);
			temRecord->currPathLength   = currPathLength;
			temRecord->comPathLength    = completePath->completeLength;
			temRecord->numReferDiscrete = referencePath->numOfReferPoint;
			temRecord->referPathSize    = referencePath->edges.size();
			
			int     timeOutFlag = 0;
			clock_t startTime = clock();
			bool queryResultFlag = roadSimpleQueryPath(myGraph, referencePath, currentPath, temPath, temRecord, currentVertex, &timeOutFlag, &temDistance);
			if (queryResultFlag == false)
			{
				if (timeOutFlag == 1)
				{
					timeFlag = 1;
					break;
				}
				else
				{
					timeOutFlag = 0;
					bool queryResultFlagWithLoop = roadSimleQueryPathWithLoop(myGraph, referencePath, currentPath, temPath, temRecord, currentVertex, &timeOutFlag, &temDistance);
					if (queryResultFlagWithLoop == false)
					{
						if (timeOutFlag == 1)
						{
							timeFlag = 1;
							break;
						}
						else
						{
							clock_t stopTime = clock();
							temRecord->runTime = (double)(stopTime - startTime) / CLOCKS_PER_SEC;
							record[recordPos++] = (*temRecord);
						}
					}
					else
					{
						clock_t stopTime = clock();
						temRecord->runTime = (double)(stopTime - startTime) / CLOCKS_PER_SEC;
						record[recordPos++] = (*temRecord);
					}
				}

			}
			else
			{
				clock_t stopTime = clock();
				temRecord->runTime = (double)(stopTime - startTime) / CLOCKS_PER_SEC;
				record[recordPos++] = (*temRecord);
			}
		}
		if (timeFlag == 1)
		{
			count++;
			continue;	
		}
		string writeFilePath = outputFolder + "/" + strCount + "roadpartial.txt";
		writeSingleRecordToFile(record, recordPos, writeFilePath);
		
		delete referencePath;
		delete completePath;
		count++;
	}
}

/**
 * Continuous query in Road network space
 * @param myGraph graph for query
 * @param inputFloder folder path when reading files
 * @param outputFolder folder path when writing files
 * @param startCount the start position of the query
 * @param stopCount the stop position of the querys
*/
void roadContinuousQuery(Graph* myGraph, string inputFolder, string outputFolder, const int startCount, const int stopCount)
{
	int count = startCount;
	while (count <= stopCount)
	{
		printf("Start %dth query...\n", count);
		string strCount = changeIntToStr(count);
		ReferencePath* referencePath = new ReferencePath;
		CompletePath*  completePath  = new CompletePath;
		setQueryEnv(myGraph, referencePath, completePath, inputFolder, strCount);
		
		int    currentVertex = completePath->initialVertex;
		int    recordPos = 0;
		int    timeFlag  = 0;
		int    ifCalFlag = 0;
		Record record[completePath->edges.size()];
		double currPathLength = 0.0;
		double boundDistance  = 0.0;
		double pathDistance;
		vector<Edge> temPath;
		vector<Edge> lastPath;
		initializeLastPath(referencePath, lastPath);
		
		clock_t startTime = clock();
		for (unsigned int i = 1; i < completePath->edges.size(); i++)
		{
			ifCalFlag = 0;
			currPathLength += completePath->edges[i - 1].realLength;
			if (completePath->edges[i - 1].firstVertexID == currentVertex)
			{
				currentVertex = completePath->edges[i - 1].secVertexID;
			}
			else
			{
				currentVertex = completePath->edges[i - 1].firstVertexID;
			}

			vector<Edge> currentPath;
			setCurrPath(myGraph, &currentPath, completePath, i);
			
			Record* temRecord = new Record;
			temRecord->ratioNumEdge = ((double)(i)) / completePath->edges.size();
			temRecord->runTime = 0;
			temRecord->comPathSize = completePath->edges.size();
			temRecord->numOfOutPriQueue = 0;
			temRecord->referPathLength = referencePath->referenceLength;
			temRecord->ratioPathLength = currPathLength / (completePath->completeLength);
			temRecord->currPathLength = currPathLength;
			temRecord->comPathLength = completePath->completeLength;
			record[recordPos++] = (*temRecord);

			if (lastPath[i - 1].edgeID == currentPath[i - 1].edgeID)
			{
				continue;
			}
			else
			{
				int firstVertexID;
				int secVertexID;
				int endVertexID;
				if (currentPath[i - 1].firstVertexID == currentVertex)
				{
					firstVertexID = currentPath[i - 1].secVertexID;
					secVertexID   = currentPath[i - 1].firstVertexID;
				}
				else
				{
					firstVertexID = currentPath[i - 1].firstVertexID;
					secVertexID   = currentPath[i - 1].secVertexID;
				}													
				vector<Edge> localPath;
				maxmalOverlapPath(myGraph, localPath, lastPath, firstVertexID, secVertexID, &endVertexID);
				if (localPath.size() == 0)
				{
					printf("localPath's size is 0\n");
					int  timeOutFlag = 0;
					bool queryResultFlag = roadSimpleQueryPath(myGraph, referencePath, currentPath, temPath, temRecord, currentVertex, &timeOutFlag, &pathDistance);
					if (queryResultFlag == false)
					{
						if (timeOutFlag == 1)
						{
							timeFlag = 1;
							break;
						}
						else
						{
							timeOutFlag = 0;
							bool queryResultFlagWithLoop = roadSimleQueryPathWithLoop(myGraph, referencePath, currentPath, temPath, temRecord, currentVertex, &timeOutFlag, &pathDistance);
							if (queryResultFlagWithLoop == false)
							{
								if (timeOutFlag == 1)
								{
									timeFlag = 1;
								}
								break;
							}
							else
							{
								updateLastPath(lastPath, temPath, &boundDistance, &pathDistance);
							}
						}
					}
					else
					{
						updateLastPath(lastPath, temPath, &boundDistance, &pathDistance);
					}
				}
				else
				{
					printf("localPath's size is not 0\n");
					vector<Edge> localLastPath;
					int edgePos;
					for (edgePos = i - 1; edgePos < lastPath.size(); edgePos++)
					{
						int temFirstVertexID = lastPath[edgePos].firstVertexID;
						int temSecVertexID   = lastPath[edgePos].secVertexID;
						if (temFirstVertexID == endVertexID || temSecVertexID == endVertexID)
						{
							localLastPath.push_back(lastPath[edgePos]);
							break;
						}
						localLastPath.push_back(lastPath[edgePos]);
					}
					double temDistance = dfdBetweenTwoLocalPaths(myGraph, localPath, localLastPath, firstVertexID);
					boundDistance += temDistance;
					if (boundDistance <= myGraph->delta)
					{
						replacePath(lastPath, localPath, i - 1, edgePos + 1);
					}
					else
					{
						int  timeOutFlag = 0;
						bool queryResultFlag = roadSimpleQueryPath(myGraph, referencePath, currentPath, temPath, temRecord, currentVertex, &timeOutFlag, &pathDistance);
						if (queryResultFlag == false)
						{
							if (timeOutFlag == 1)
							{
								timeFlag = 1;
								break;
							}
							else
							{
								timeOutFlag = 0;
								bool queryResultFlagWithLoop = roadSimleQueryPathWithLoop(myGraph, referencePath, currentPath, temPath, temRecord, currentVertex, &timeOutFlag, &pathDistance);
								if (queryResultFlagWithLoop == false)
								{
									if (timeOutFlag == 1)
									{
										timeFlag = 1;
									}
									break;
								}
								else
								{
									updateLastPath(lastPath, temPath, &boundDistance, &pathDistance);
								}
							}
						}
						else
						{
							updateLastPath(lastPath, temPath, &boundDistance, &pathDistance);
						}
					}
				}
			}
		}
		clock_t stopTime = clock();
		if (timeFlag == 1)
		{
			count++;
			continue;
		}
		string writeFilePath = outputFolder + "/" + strCount + "roadContinuousQuery.txt";
		writeContinuousRecordToFile(record, recordPos, writeFilePath, (double)(stopTime - startTime) / CLOCKS_PER_SEC);
		
		delete referencePath;
		delete completePath;
		count++;
	}
}
