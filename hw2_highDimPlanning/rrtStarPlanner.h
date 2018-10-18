#ifndef RRT_STAR_PLANNER
#define RRT_STAR_PLANNER

#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <vector>
#include <cfloat>
#include <iostream> 
#include <math.h>
#include <utility>
#include "helperFunc.h"

#if !defined(GETMAPINDEX)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)
#endif

//the length of each link in the arm (should be the same as the one used in runtest.m)
#if !defined(LINKLENGTH_CELLS)
#define LINKLENGTH_CELLS 10
#endif

#if !defined(PI)
#define PI 3.141592654
#endif

using namespace std;

/*-----------------------------------------
------- a class of rrt-star planner -------
------------------------------------------*/
class rrtStarPlanner : public rrtPlanner{
public:
	rrtStarPlanner(double* mapIn, int dimIn, int xIn, int yIn, rrtNode* rNode1, rrtNode* rNode2, double step);
	void updateG(rrtNode* rNode);
	void rewireNode(rrtNode* rNode);
	vector<rrtNode*> planRRTStar(double minDim[], double maxDim[]);
};

rrtStarPlanner::rrtStarPlanner(double* mapIn, int dimIn, int xIn, int yIn, rrtNode* rNode1, rrtNode* rNode2, double step)
			: rrtPlanner(mapIn, dimIn, xIn, yIn, rNode1, rNode2, step) {}

// set the gValue of node in tree
void rrtStarPlanner::updateG(rrtNode* rNode) {
	rrtNode* parentNode = rNode->getParent();
	if (parentNode==NULL || rNode == NULL) return;
	double d = disNode(parentNode, rNode);
	rNode->setGValue(parentNode->getGValue()+d);
}

// try to rewire node in tree
void rrtStarPlanner::rewireNode(rrtNode* rNode) {
	// find nearest nodes in tree
	vector<rrtNode*> nearest = findNN(rNode, 0.5);
	// store the original parent and gValue
	rrtNode* parent = rNode->getParent();
	double gRecord = rNode->getGValue();
	// check if new g value would be better
	double newG;
	for (int i=0; i<nearest.size(); i++) {
		newG = nearest[i]->getGValue() + disNode(nearest[i], rNode);
		if (newG < gRecord) {
			gRecord = newG;
			parent = nearest[i];
		}
	}
	rNode->setGValue(newG);
	rNode->setParent(parent);
}

// planRRTStar
vector<rrtNode*> rrtStarPlanner::planRRTStar(double minDim[], double maxDim[]) {
	vector<rrtNode*> recordPath;
	if (start == NULL || goal == NULL) return recordPath;
	
	// the start node has a gValue of 0.0
	start->setGValue(0.0);
	
	int count = 0;
	while(1) {
		if (count>30000) {
			break;			
		}
		// generate a random node, try to extend a step
		rrtNode* tmpt = randNewNode(minDim, maxDim);
		if (extend(tmpt)) {
			// if successful, updateG, rewire
			updateG(treeStart.back());
			rewireNode(treeStart.back());
		}
		delete tmpt;
		// every twenty times, try to extend to goal
		if ((count%20)==0) {
			extend(goal);
		}
		/*----------------if goal reached ,break--------------------*/
		if (isEqual(goal, treeStart.back())) {
			mexPrintf("\n\nreach goal!!!\n\n");
			rrtNode* backTrackNode = treeStart.back();
			while (1) {
				recordPath.push_back(backTrackNode);
				if (isEqual(backTrackNode, start)) {break;}
				backTrackNode = backTrackNode->getParent();
				// // add interpolate points
				// vector<rrtNode*> middle = middleNode(recordPath.back(), backTrackNode);
				// recordPath.insert(recordPath.end(),middle.begin(),middle.end()); 
			}
			break;
		}
		count++;
	}
	
	mexPrintf("tree size: %d\n", treeStart.size());
	return recordPath;
}


#endif // RRT_STAR_PLANNER