// PathFinding.cpp : Defines the entry point for the console application.
// VS2013 - Win32

#include "stdafx.h"
#include <vector>
#include <iostream>
#include <mutex>

#define INDEX_NULL -1
#define INTEGER_MAX 0x7FFFFFFF

std::mutex simple_mutex;

// Converts a 1D point coordinate into a 2D coordinate
void OneDToTwoD(const int width, const int index, int& xResult, int& yResult)
{
	xResult = index % width;
	yResult = index / width;
}

// Converts a 2D point coordinate into a 1D coordinate
void TwoDToOneD(const int width, const int x, const int y, int& result)
{
	result = (width * y) + x;
}

// Fills out a vector with current position neighbors
void NeighborsOneD(const int current, const unsigned char* pMap, const int nMapWidth, const int nMapHeight, std::vector<int>& neighbors)
{
	int xNeighbour;
	int yNeighbour;
	OneDToTwoD(nMapWidth, current, xNeighbour, yNeighbour);
	if (xNeighbour - 1 >= 0)
	{
		int neighbour;
		TwoDToOneD(nMapWidth, xNeighbour - 1, yNeighbour, neighbour);
		neighbors.push_back(neighbour);
	}
	if (xNeighbour + 1 < nMapWidth)
	{
		int neighbour;
		TwoDToOneD(nMapWidth, xNeighbour + 1, yNeighbour, neighbour);
		neighbors.push_back(neighbour);
	}
	if (yNeighbour - 1 >= 0)
	{
		int neighbour;
		TwoDToOneD(nMapWidth, xNeighbour, yNeighbour - 1, neighbour);
		neighbors.push_back(neighbour);
	}
	if (yNeighbour + 1 < nMapHeight)
	{
		int neighbour;
		TwoDToOneD(nMapWidth, xNeighbour, yNeighbour + 1, neighbour);
		neighbors.push_back(neighbour);
	}
}

// (Basic heuristic) Returns the Manhattan distance between two points in 2D coordinates
int Manhattan(const int x0, const int y0, const int x1, const int y1)
{
	return abs(x1 - x0) + abs(y1 - y0);
}

// (Basic heuristic) Returns the Manhattan distance between two points in 1D coordinates
int Manhattan(const int width, const int i, const int j)
{
	int xStart;
	int yStart;
	int xTarget;
	int yTarget;
	OneDToTwoD(width, i, xStart, yStart);
	OneDToTwoD(width, j, xTarget, yTarget);
	return Manhattan(xStart, yStart, xTarget, yTarget);
}

// A* implementation
int FindPath(
	const int nStartX,
	const int nStartY,
	const int nTargetX,
	const int nTargetY,
	const unsigned char* pMap,
	const int nMapWidth,
	const int nMapHeight,
	int* pOutBuffer,
	const int nOutBufferSize)
{
	// Lock critical section (verrrry basic way)
	simple_mutex.lock();

	// If Start equals target path size is 0
	if (nStartX == nTargetX && nStartY == nTargetY)
	{
		return 0;
	}
	// Define the open and the close list
	std::vector<int> openList;
	std::vector<int> closedList;

	// Cost and heuristic buffers
	const int mapSize = nMapWidth * nMapHeight;
	int* costSoFar = new int[mapSize];
	int* hEstimate = new int[mapSize];
	int* camefrom = new int[mapSize];

	// Initialize path history
	for (int i = 0; i < mapSize; ++i)
	{
		camefrom[i] = INDEX_NULL;
	}

	int indexStart;
	int indexEnd;
	TwoDToOneD(nMapWidth, nStartX, nStartY, indexStart);
	TwoDToOneD(nMapWidth, nTargetX, nTargetY, indexEnd);
	costSoFar[indexStart] = 0;
	hEstimate[indexStart] = Manhattan(nStartX, nStartY, nTargetX, nTargetY);

	// Add the start point to the openList
	openList.push_back(indexStart);
	int current = openList[0];
	bool success = false;
	// Release the Kraken !
	while (openList.size() > 0)
	{
		// Look for the smallest cost in the openList and make it the current point
		int best = INTEGER_MAX;
		const int openListSize = openList.size();
		int i = 0;
		for (; i < openListSize; ++i)
		{
			int cost = hEstimate[openList[i]];
			if (cost < best)
			{
				current = openList[i];
				best = cost;
			}
		}
		// Remove the current point from the openList
		openList.erase(openList.begin() + (i - 1));
		// Add the current point to the closedList
		closedList.push_back(current);
		// Stop if we reached the end
		if (current == indexEnd)
		{
			success = true;
			break;
		}

		// Go over valid neighbors and get the most interesting
		std::vector<int> neighbors;
		NeighborsOneD(current, pMap, nMapWidth, nMapHeight, neighbors);
		const int neighborsSize = neighbors.size();
		i = 0;
		for (; i < neighborsSize; ++i)
		{
			const int currentNeighbor = neighbors[i];
			// If accessible and not already kicked out (could be done in Neighbors research function)
			if ((int)pMap[currentNeighbor] == 1
				&& std::find(closedList.begin(), closedList.end(), currentNeighbor) == closedList.end())
			{
				// Evaluate cost and add to the open list
				const int cost = costSoFar[current] + Manhattan(nMapWidth, current, currentNeighbor);
				if (std::find(openList.begin(), openList.end(), currentNeighbor) == openList.end())
				{
					openList.push_back(currentNeighbor);
				}
				// Else if already in open and is more expensive than before, forget it.
				else if (cost >= costSoFar[currentNeighbor])
				{
					continue;
				}
				costSoFar[currentNeighbor] = cost;
				hEstimate[currentNeighbor] = cost + Manhattan(nMapWidth, currentNeighbor, indexEnd);
				camefrom[current] = currentNeighbor;
			}
		}
	}
	// If we reached target, fill out output buffer
	int pathIndex = 0;
	if (success)
	{
		for (int idx = 0; camefrom[idx] != INDEX_NULL; ++idx)
		{
			pOutBuffer[pathIndex] = camefrom[idx];
			++pathIndex;
		}
		pOutBuffer[pathIndex] = current;
	}
	
	//Clean up dynamic alloc
	delete hEstimate;
	delete camefrom;
	delete costSoFar;

	//Unlock critical section
	simple_mutex.unlock();

	return success ? pathIndex + 1 : INDEX_NULL;
}

// Basic output function
void Output(const int* pOutBuffer, const int bufferSize)
{
	std::cout << "------------------------------------------ \n";
	if (bufferSize > 0)
	{
		std::cout << "Path size = " << bufferSize << "\n";
		std::cout << "[";
		for (unsigned int i = 0; i < bufferSize - 1; ++i)
		{
			std::cout << pOutBuffer[i] << ", ";
		}
		std::cout << pOutBuffer[bufferSize - 1] << "] \n";
	}
	else
	{
		std::cout << "Couldn't find path :{ \n";
	}
	
}

int _tmain(int argc, _TCHAR* argv[])
{
	unsigned char pMap[] = { 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1 };
	int pOutBuffer[12];
	const int result1 = FindPath(0, 0, 1, 2, pMap, 4, 3, pOutBuffer, 12);
	Output(pOutBuffer, result1);

	unsigned char pMap2[] = { 0, 0, 1, 0, 1, 1, 1, 0, 1 };
	int pOutBuffer2[7];
	const int result2 = FindPath(2, 0, 0, 2, pMap2, 3, 3, pOutBuffer2, 7);
	Output(pOutBuffer, result2);

	system("PAUSE");
	return 0;
}

