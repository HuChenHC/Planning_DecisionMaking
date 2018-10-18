#ifndef RRT_PLANNER
#define RRT_PLANNER

#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <vector>
#include <cfloat>
#include <iostream> 
#include <math.h>
#include <utility>
#include "rrtNode.h"
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
-------- a class of rrt planner -----------
------------------------------------------*/
class rrtPlanner{
protected:
	double* map;
	vector<rrtNode*> treeStart;
	rrtNode* start;
	rrtNode* goal;
	int dim, xSize, ySize;
	double stepSize;
public:
	rrtPlanner();
	rrtPlanner(double* mapIn, int dimIn, int xIn, int yIn, rrtNode* rNode1, rrtNode* rNode2, double step);
	~rrtPlanner();
	void clearTree();
	void clearStartGoal();
	double disNode(rrtNode* rNode1, rrtNode* rNode2);
	rrtNode* randNewNode(double minDim[], double maxDim[]);
	bool nodeInCollision(rrtNode* rNode);
	bool actionInCollision(rrtNode* rNode1, rrtNode* rNode2);
	vector<rrtNode*> middleNode(rrtNode* rNode1, rrtNode* rNode2);
	rrtNode* findNN(rrtNode* rNode);
	vector<rrtNode*> findNN(rrtNode* rNode, const int k);
	vector<rrtNode*> findNN(rrtNode* rNode, const double radius);
	bool isEqual(rrtNode* rNode1, rrtNode* rNode2);
	
	int extend(rrtNode* rNode);
	vector<rrtNode*> planNow(double minDim[], double maxDim[]);
	int getTreeStartSize();
};


rrtPlanner::rrtPlanner(){dim=4;}    // for test only, don't forget to delete "fim=4;" 
// get the map in, set seed for random 
rrtPlanner::rrtPlanner(double* mapIn, int dimIn, int xIn, int yIn, rrtNode* rNode1, rrtNode* rNode2, double step) {
	srand(time(0));
	map = mapIn; 
	dim = dimIn;
	xSize = xIn;
	ySize = yIn;
	stepSize = step;
	if (!nodeInCollision(rNode1)) {
		start = rNode1;
		start->setParent(start);
		treeStart.push_back(start);
	}
	else {
		start = NULL;
	}
	if (!nodeInCollision(rNode2)) {
		goal = rNode2;
	}
	else {
		goal = NULL;
	}
	mexPrintf("\nrrt planner initialized\n");
}

// delete tree & start & goal
rrtPlanner::~rrtPlanner() {
	clearTree();
//mexPrintf("rrt planner cleaned\n");
}

// delete the pointed nodes and clear treeStart
void rrtPlanner::clearTree() {
	for (int i=1; i<(int)treeStart.size(); i++) {
		delete treeStart.at(i); treeStart.at(i)=NULL;
	}
	treeStart.clear();
}

// delete start and goal
void rrtPlanner::clearStartGoal() {
	if (start!=NULL) {
		delete start; 
		start = NULL;
	}
	if (goal!=NULL) {
		delete goal; 
		goal = NULL;
	}
}

// calculate the distance between two rrtNodes
double rrtPlanner::disNode(rrtNode* rNode1, rrtNode* rNode2) {
	double tmpt = 0;
	for (int i=0; i<(int)rNode1->getData().size(); i++) {
		tmpt += pow(rNode1->getData().at(i) - rNode2->getData().at(i), 2.0);
	}
	return pow(tmpt,0.5);
}

// return a randomly sampled node
rrtNode* rrtPlanner::randNewNode(double minDim[], double maxDim[]) {
	vector<double> data;
	// for each dim sample randomly within range
	for (int i=0; i<dim; i++) {
		data.push_back(fRand(minDim[i], maxDim[i]));
	}
	rrtNode* tmpt = new rrtNode(data);
	return tmpt;
}

// node in collision return 1, else return 0
bool rrtPlanner::nodeInCollision(rrtNode* rNode){
	// if NULL, not valid state, we say it in collision
	if (rNode == NULL) return true;
	double x0, y0, x1, y1;
	//iterate through all the links starting with the base
	x1 = ((double)xSize)/2.0;
    y1 = 0.0;
	for(int i = 0; i < dim; i++) {
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2.0*PI-rNode->getData().at(i));
		y1 = y0 - LINKLENGTH_CELLS*sin(2.0*PI-rNode->getData().at(i));
		//check the validity of the corresponding line segment
		if(lineMapCollision(x0,y0,x1,y1,map,xSize,ySize)) {
			return true;
		}
	}    
    return false; 	
}

// action between two nodes, interpolate the action into several nodes and check collision
bool rrtPlanner::actionInCollision(rrtNode* rNode1, rrtNode* rNode2) {
	// if any node is NULL, in collision
	if (rNode1 == NULL || rNode2 == NULL) return true;
	rrtNode* tmpt = new rrtNode(rNode1->getData());
	vector<double> diffBetweenNodes;
	// set number of samples to be how many (PI/20) does the action have
	double distance = 0.0;
    for (int i = 0; i<(int)rNode1->getData().size(); i++){
        if(distance < pow(1.1, (int)rNode1->getData().size()-i-1)*fabs(rNode1->getData().at(i) - rNode2->getData().at(i))) {
            distance = pow(1.1, (int)rNode1->getData().size()-i-1)*fabs(rNode1->getData().at(i) - rNode2->getData().at(i));
		}
		diffBetweenNodes.push_back(rNode2->getData().at(i) - rNode1->getData().at(i));
    }
    int numOfSamples = (int)(distance/(PI/30.0));
	// check each sample, any sample in collision, the action is in collision
	for (int i=0; i<numOfSamples; i++) {
		for (int j = 0; j<(int)rNode1->getData().size(); j++){
			tmpt->increaseData(diffBetweenNodes.at(j)/double(numOfSamples),j);
		}
		if (nodeInCollision(tmpt)) {
			delete tmpt;
			return true;
		}
	}
	// check end node
	if (nodeInCollision(rNode2)) {
		delete tmpt;
		return true;
	}
	delete tmpt;
	return false;
}

// return the vector of middle node
vector<rrtNode*> rrtPlanner::middleNode(rrtNode* rNode1, rrtNode* rNode2) {
	vector<double> diffBetweenNodes;
	// set number of samples to be how many (PI/20) does the action have
	double distance = 0.0;
    for (int i = 0; i<(int)rNode1->getData().size(); i++){
        if(distance < pow(1.1, (int)rNode1->getData().size()-i-1)*fabs(rNode1->getData().at(i) - rNode2->getData().at(i))) {
            distance = pow(1.1, (int)rNode1->getData().size()-i-1)*fabs(rNode1->getData().at(i) - rNode2->getData().at(i));
		}
		diffBetweenNodes.push_back(rNode2->getData().at(i) - rNode1->getData().at(i));
    }
    int numOfSamples = (int)(distance/(PI/15.0));	
	rrtNode* tmpt = new rrtNode(rNode1->getData());
	vector<rrtNode*> middle;
	// check each sample, any sample in collision, the action is in collision
	for (int i=0; i<numOfSamples; i++) {
		for (int j = 0; j<(int)rNode1->getData().size(); j++){
			tmpt->increaseData(diffBetweenNodes.at(j)/double(numOfSamples),j);
		}
		middle.push_back(new rrtNode(tmpt->getData()));
	}
	delete tmpt;
	return middle;
}

// find the nearest node from treeStart
rrtNode* rrtPlanner::findNN(rrtNode* rNode) {
	rrtNode* tmpt = NULL;
	double minDis = DBL_MAX;
	// check each node and save the nearest one
	for (int i=0; i<(int)treeStart.size(); i++) {
		double tmptDis = disNode(rNode, treeStart.at(i));
		// if nearer, store the node
		if (tmptDis < minDis) {
			minDis = tmptDis;
			tmpt = treeStart.at(i);
		}
	}
	return tmpt;
}

// find k nearest nodes from treeStart
vector<rrtNode*> rrtPlanner::findNN(rrtNode* rNode, const int k) {
	vector<rrtNode*> tmpt;
	vector<double> minDis;
	if (k>0) {
		for (int i = 0; i<k; i++) {
			minDis.push_back(DBL_MAX);
		}
		// check each node and save the nearest one
		for (int i=0; i<(int)treeStart.size(); i++) {
			double tmptDis = disNode(rNode, treeStart.at(i));
			// if nearer, store the node
			if (tmptDis < minDis[k-1]) {
				for (int j=0; j<k; j++) {
					if (tmptDis<minDis[j]) {
						minDis.insert(minDis.begin()+j, tmptDis);
						minDis.erase(minDis.end()-1);
						tmpt.insert(tmpt.begin()+j, treeStart.at(i));
						if ((int)tmpt.size()>k) {
							tmpt.erase(tmpt.end()-1);
						}
						break;
					}
				}
			}
		}
	}
	return tmpt;
}

// find nodes within radius
vector<rrtNode*> rrtPlanner::findNN(rrtNode* rNode, const double radius) {
	vector<rrtNode*> tmpt;
	if (radius>0) {
		// check each node and save
		for (int i=0; i<(int)treeStart.size(); i++) {
			double tmptDis = disNode(rNode, treeStart.at(i));
			// if nearer, store the node
			if (tmptDis < radius) {
				tmpt.push_back(treeStart.at(i));
			}
		}
	}
	return tmpt;
}

// check if two nodes at same position
bool rrtPlanner::isEqual(rrtNode* rNode1, rrtNode* rNode2) {
	// if any node is NULL, not the same
	if (rNode1 == NULL || rNode2 == NULL) return false;
	// check every dim, any not same, not same point
	for (int i=0; i<(int)rNode1->getData().size(); i++) {
		if (rNode1->getData().at(i) != rNode2->getData().at(i)){
			return false;
		}
	}
	return true;
}

// given a node, try to extend the nearest node toward new node for stepSize 
int rrtPlanner::extend(rrtNode* rNode) {
	// find nearest node from tree, if found, extend
	rrtNode* nearest = findNN(rNode);
	if (nearest!=NULL) {
		bool limited = false;
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
			treeStart.push_back(extended);
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

// rrt planner, return the path from goal to start
vector<rrtNode*> rrtPlanner::planNow(double minDim[], double maxDim[]) {
	vector<rrtNode*> recordPath;
	if (start == NULL || goal == NULL) return recordPath;
	
	int count = 0;
	while(1) {
		if (count>30000) {
			break;			
		}
		// generate a random node, try to extend a step
		rrtNode* tmpt = randNewNode(minDim, maxDim);
		extend(tmpt);
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

// return size of tree from start point 
int rrtPlanner::getTreeStartSize() {
	return treeStart.size();
}


#endif