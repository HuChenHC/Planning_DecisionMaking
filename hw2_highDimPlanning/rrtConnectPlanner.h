#ifndef RRT_CONNECT_PLANNER
#define RRT_CONNECT_PLANNER

#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <vector>
#include <cfloat>
#include <iostream> 
#include <math.h>
#include <utility>
#include "rrtNode.h"
#include "rrtPlanner.h"
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
----- a class of rrt-connect planner ------
------------------------------------------*/
class rrtConnectPlanner : public rrtPlanner {
private:
	vector<rrtNode*> treeGoal;
public:
	rrtConnectPlanner(double* mapIn, int dimIn, int xIn, int yIn, 
				rrtNode* rNode1, rrtNode* rNode2, double step);
	rrtNode* findNNInTreeGoal(rrtNode* rNode);
	int extendFromTreeGoal(rrtNode* rNode);	
	bool connect(rrtNode* rNode, bool fromStart);
	vector<rrtNode*> planRRTConnect(double minDim[], double maxDim[]);
};



// also add goal to treeGoal
rrtConnectPlanner::rrtConnectPlanner(double* mapIn, int dimIn, int xIn, int yIn, 
				rrtNode* rNode1, rrtNode* rNode2, double step) 
				: rrtPlanner(mapIn, dimIn, xIn, yIn, rNode1, rNode2, step) {
	if (!nodeInCollision(rNode2)) {
		goal->setParent(goal);
		treeGoal.push_back(goal);
	}
}

// find the nearest node from treeGoal
rrtNode* rrtConnectPlanner::findNNInTreeGoal(rrtNode* rNode) {
	rrtNode* tmpt = NULL;
	double minDis = DBL_MAX;
	// check each node and save the nearest one
	for (int i=0; i<(int)treeGoal.size(); i++) {
		double tmptDis = disNode(rNode, treeGoal.at(i));
		// if nearer, store the node
		if (tmptDis < minDis) {
			minDis = tmptDis;
			tmpt = treeGoal.at(i);
		}
	}
	return tmpt;
}

// one step extension toward nearest in treeGoal
int rrtConnectPlanner::extendFromTreeGoal(rrtNode* rNode) {
	// find nearest node from tree, if found, extend
	rrtNode* nearest = findNNInTreeGoal(rNode);
	if (nearest!=NULL) {
		bool limited =false;
		// get the direction
		vector<double> diffBetweenNodes;
		for (int i = 0; i<(int)rNode->getData().size(); i++){
			diffBetweenNodes.push_back(rNode->getData().at(i) - nearest->getData().at(i));
		}
		// factor to multiply the direction
		double factor = 1.0;
		if (disNode(rNode, nearest)!=0) {
			factor = stepSize/disNode(rNode, nearest);
			limited = true;
		}
		if (factor > 1.0) {
			factor = 1.0;
			limited = false;
		}		
		// add to position of nearest node 
		rrtNode* extended = new rrtNode(nearest->getData());
		for (int i = 0; i<(int)rNode->getData().size(); i++){
			extended->increaseData(diffBetweenNodes.at(i)*factor,i);
		}
		// check if actionInCollision between nearest and extended, if not, add to tree
		if (!actionInCollision(nearest, extended)) {
			extended->setParent(nearest);
			treeGoal.push_back(extended);
			if (limited) {
				// extended
				return 1;
			}
			else {
				// connected
				return 2;
			}
		}
		// if not added to tree, delete it
		delete extended;
	}
	return 0;
}

// given a node, true for reached, false for cannot move on
bool rrtConnectPlanner::connect(rrtNode* rNode, bool fromStart) {
	if (fromStart) {
		while (1) {
			int test = extend(rNode);
			if (test == 0) return false;
			if (test == 2) return true;	
		}
	}
	else {
		while (1) {
			int test = extendFromTreeGoal(rNode);
			if (test == 0) return false;
			if (test == 2) return true;	
		}
	}
}

// rrt-connect planner, return the path from goal to start
vector<rrtNode*> rrtConnectPlanner::planRRTConnect(double minDim[], double maxDim[]) {
	vector<rrtNode*> recordPath;
	if (start == NULL || goal == NULL) return recordPath;
	
	bool fromStart = true, reached = false;
	
	int count = 0;
	while(1) {
		if (count>20000) {
			break;			
		}
		// generate a random node, try to extend
		rrtNode* tmpt = randNewNode(minDim, maxDim);
		// extend from treeStart
		if (fromStart) {
			// if successfully extended from treeStart, try to connect treeGoal
			if (extend(tmpt)>0) {
				// connect to see if reached
				if (connect(treeStart.back(), !fromStart)) {
					reached = true;
				}
			}
		}
		// extend from treeGoal
		else {
			// if successfully extended from treeGoal, try to connect treeStart
			if (extendFromTreeGoal(tmpt)>0) {
				// connect to see if reached
				if (connect(treeGoal.back(), !fromStart)) {
					reached = true;
				}
			}
		}
		delete tmpt;
		
		/*----------------if goal reached ,break--------------------*/
		if (reached) {
			mexPrintf("\n\ntrees connected!!!\n\n");
			// push nodes from treeStart
			rrtNode* backTrackNode = treeStart.back();
			while (1) {
				recordPath.push_back(backTrackNode);
				if (isEqual(backTrackNode, start)) {break;}
				backTrackNode = backTrackNode->getParent();
				// // add interpolate points
				// vector<rrtNode*> middle = middleNode(recordPath.back(), backTrackNode);
				// recordPath.insert(recordPath.end(),middle.begin(),middle.end()); 
			}
			// then push nodes from treeGoal, pushing from the front
			backTrackNode = treeGoal.back();
			backTrackNode = backTrackNode->getParent();
			while (1) {
				recordPath.insert(recordPath.begin(), backTrackNode);
				if (isEqual(backTrackNode, goal)) {break;}
				backTrackNode = backTrackNode->getParent(); 
			}
			break;
		}
		fromStart = !fromStart;
		count++;
	}
	
	mexPrintf("tree size: %d, %d\n", treeStart.size(), treeGoal.size());
	return recordPath;
}

#endif // RRT_CONNECT_PLANNER