#ifndef PRM_PLANNER
#define PRM_PLANNER

#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <vector>
#include <cfloat>
#include <iostream> 
#include <math.h>
#include <set>
#include <utility>
#include <unordered_map>
#include "mex.h"
#include "prmNode.h"
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
-------- a class of prm planner -----------
------------------------------------------*/
class prmPlanner{
private:
	double* map;
	vector<prmNode*> roadMap;
	prmNode* start;
	prmNode* goal;
	int dim, xSize, ySize;
public:
	prmPlanner(double* mapIn, int dimIn, int xIn, int yIn);
	~prmPlanner();
	void clearRoadMap();
	void clearStartGoal();
	double disNode(prmNode* pNode1, prmNode* pNode2);
	prmNode* randNewNode(double minDim[], double maxDim[]);
	bool nodeInCollision(prmNode* pNode);
	bool actionInCollision(prmNode* pNode1, prmNode* pNode2);
	vector<prmNode*> middleNode(prmNode* pNode1, prmNode* pNode2);
	prmNode* findNN(prmNode* pNode);
	vector<prmNode*> findNN(prmNode* pNode, const int k);
	vector<prmNode*> findNN(prmNode* pNode, const double radius);
	bool isEqual(prmNode* pNode1, prmNode* pNode2);
	void linkNode(prmNode* pNode1, prmNode* pNode2);
	void addStartGoal(prmNode* pNode1, prmNode* pNode2);
	void geneRoadMap(int nNode, double minDim[], double maxDim[]);
	double calHeu(prmNode* pNode);
	vector<prmNode*> planAStar();
	int getRoadMapSize();
};

// get the map in, set seed for random 
prmPlanner::prmPlanner(double* mapIn, int dimIn, int xIn, int yIn) {
	mexPrintf("prm planner initialized\n");
	srand(time(0));
	map = mapIn; 
	dim = dimIn;
	xSize = xIn;
	ySize = yIn;
	start = NULL;
	goal = NULL;
}

// delete map & start & goal
prmPlanner::~prmPlanner() {
	clearRoadMap();
	clearStartGoal();
	mexPrintf("\nprm planner cleaned\n");
}

// delete the pointed nodes and clear roadMap
void prmPlanner::clearRoadMap() {
	for (int i=0; i<(int)roadMap.size(); i++) {
		delete roadMap.at(i); roadMap.at(i)=NULL;
	}
	roadMap.clear();
}

// delete start and goal
void prmPlanner::clearStartGoal() {
	if (start!=NULL) {
		delete start; 
		start = NULL;
	}
	if (goal!=NULL) {
		delete goal; 
		goal = NULL;
	}
}


// calculate the distance between two prmNodes
double prmPlanner::disNode(prmNode* pNode1, prmNode* pNode2) {
	double tmpt = 0;
	for (int i=0; i<(int)pNode1->getData().size(); i++) {
		tmpt += pow(pNode1->getData().at(i) - pNode2->getData().at(i), 2.0);
	}
	return pow(tmpt,0.5);
}
// double prmPlanner::disNode(prmNode* pNode1, prmNode* pNode2) {
	// double tmpt = 0;
	// for (int i=0; i<(int)pNode1->getData().size(); i++) {
		// tmpt += pow(pNode1->getData().at(i) - pNode2->getData().at(i), 2.0);
	// }
	// return pow(tmpt,0.5);
	// double x0, y0, x1, y1, x2, y2;
	// x1 = ((double)xSize)/2.0;
    // y1 = 0.0;
	// for(int i = 0; i < dim; i++) {
		// //compute the corresponding line segment
		// x0 = x1;
		// y0 = y1;
		// x1 = x0 + LINKLENGTH_CELLS*cos(2.0*PI-pNode1->getData().at(i));
		// y1 = y0 - LINKLENGTH_CELLS*sin(2.0*PI-pNode1->getData().at(i));
	// } 
	// // store end of pNode1
	// x2 = x1; 
	// y2 = y1;
	// x1 = ((double)xSize)/2.0;
    // y1 = 0.0;
	// for(int i = 0; i < dim; i++) {
		// //compute the corresponding line segment
		// x0 = x1;
		// y0 = y1;
		// x1 = x0 + LINKLENGTH_CELLS*cos(2.0*PI-pNode2->getData().at(i));
		// y1 = y0 - LINKLENGTH_CELLS*sin(2.0*PI-pNode2->getData().at(i));
	// } 
	// return pow(pow(x1-x2, 2.0)+pow(y1-y2, 2.0), 0.5);
// }

// return a randomly sampled node
prmNode* prmPlanner::randNewNode(double minDim[], double maxDim[]) {
	vector<double> data;
	// for each dim sample randomly within range
	for (int i=0; i<dim; i++) {
		data.push_back(fRand(minDim[i], maxDim[i]));
	}
	prmNode* tmpt = new prmNode(data);
	return tmpt;
}

// node in collision return 1, else return 0
bool prmPlanner::nodeInCollision(prmNode* pNode){
	// if NULL, not valid state, we say it in collision
	if (pNode == NULL) return true;
	double x0, y0, x1, y1;
	//iterate through all the links starting with the base
	x1 = ((double)xSize)/2.0;
    y1 = 0.0;
	for(int i = 0; i < dim; i++) {
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + double(LINKLENGTH_CELLS)*cos(2.0*PI-pNode->getData().at(i));
		y1 = y0 - double(LINKLENGTH_CELLS)*sin(2.0*PI-pNode->getData().at(i));
		//check the validity of the corresponding line segment
		if(lineMapCollision(x0,y0,x1,y1,map,xSize,ySize)) {
			return true;
		}
	}    
    return false; 	
}

// action between two nodes, interpolate the action into several nodes and check collision
bool prmPlanner::actionInCollision(prmNode* pNode1, prmNode* pNode2) {
	// if any node is NULL, in collision
	if (pNode1 == NULL || pNode2 == NULL) return true;
	prmNode* tmpt = new prmNode(pNode1->getData());
	vector<double> diffBetweenNodes;
	// set number of samples to be how many (PI/20) does the action have
	double distance = 0.0;
    for (int i = 0; i<(int)pNode1->getData().size(); i++){
        if(distance < pow(1.1, (int)pNode1->getData().size()-i-1)*fabs(pNode1->getData().at(i) - pNode2->getData().at(i))) {
            distance = pow(1.1, (int)pNode1->getData().size()-i-1)*fabs(pNode1->getData().at(i) - pNode2->getData().at(i));
		}
		diffBetweenNodes.push_back(pNode2->getData().at(i) - pNode1->getData().at(i));
    }
    int numOfSamples = (int)(distance/(PI/20.0));
	// check each sample, any sample in collision, the action is in collision
	for (int i=0; i<numOfSamples; i++) {
		for (int j = 0; j<(int)pNode1->getData().size(); j++){
			tmpt->increaseData(diffBetweenNodes.at(j)/double(numOfSamples),j);
		}
		if (nodeInCollision(tmpt)) {
			delete tmpt;
			return true;
		}
	}
	// check end node
	if (nodeInCollision(pNode2)) {
		delete tmpt;
		return true;
	}
	delete tmpt;
	return false;
}

// return the vector of middle node
vector<prmNode*> prmPlanner::middleNode(prmNode* pNode1, prmNode* pNode2) {
	vector<double> diffBetweenNodes;
	// set number of samples to be how many (PI/20) does the action have
	double distance = 0.0;
    for (int i = 0; i<(int)pNode1->getData().size(); i++){
        if(distance < pow(1.1, (int)pNode1->getData().size()-i-1)*fabs(pNode1->getData().at(i) - pNode2->getData().at(i))) {
            distance = pow(1.1, (int)pNode1->getData().size()-i-1)*fabs(pNode1->getData().at(i) - pNode2->getData().at(i));
		}
		diffBetweenNodes.push_back(pNode2->getData().at(i) - pNode1->getData().at(i));
    }
    int numOfSamples = (int)(distance/(PI/15.0));	
	prmNode* tmpt = new prmNode(pNode1->getData());
	vector<prmNode*> middle;
	// check each sample, any sample in collision, the action is in collision
	for (int i=0; i<numOfSamples; i++) {
		for (int j = 0; j<(int)pNode1->getData().size(); j++){
			tmpt->increaseData(diffBetweenNodes.at(j)/double(numOfSamples),j);
		}
		middle.push_back(new prmNode(tmpt->getData()));
	}
	delete tmpt;
	return middle;
}

// find the nearest node from roadMap
prmNode* prmPlanner::findNN(prmNode* pNode) {
	prmNode* tmpt = NULL;
	double minDis = DBL_MAX;
	// check each node and save the nearest one
	for (int i=0; i<(int)roadMap.size(); i++) {
		double tmptDis = disNode(pNode, roadMap.at(i));
		// if nearer, store the node
		if (tmptDis < minDis) {
			minDis = tmptDis;
			tmpt = roadMap.at(i);
		}
	}
	return tmpt;
}

// find k nearest nodes from roadMap
vector<prmNode*> prmPlanner::findNN(prmNode* pNode, const int k) {
	vector<prmNode*> tmpt;
	vector<double> minDis;
	if (k>0) {
		for (int i = 0; i<k; i++) {
			minDis.push_back(DBL_MAX);
		}
		// check each node and save the nearest one
		for (int i=0; i<(int)roadMap.size(); i++) {
			double tmptDis = disNode(pNode, roadMap.at(i));
			// if nearer, store the node
			if (tmptDis < minDis[k-1]) {
				for (int j=0; j<k; j++) {
					if (tmptDis<minDis[j]) {
						minDis.insert(minDis.begin()+j, tmptDis);
						minDis.erase(minDis.end()-1);
						tmpt.insert(tmpt.begin()+j, roadMap.at(i));
						if (tmpt.size()>k) {
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
vector<prmNode*> prmPlanner::findNN(prmNode* pNode, const double radius) {
	vector<prmNode*> tmpt;
	if (radius>0) {
		// check each node and save
		for (int i=0; i<(int)roadMap.size(); i++) {
			double tmptDis = disNode(pNode, roadMap.at(i));
			// if nearer, store the node
			if (tmptDis < radius) {
				tmpt.push_back(roadMap.at(i));
			}
		}
	}
	return tmpt;
}

// check if two nodes at same position
bool prmPlanner::isEqual(prmNode* pNode1, prmNode* pNode2) {
	// if any node is NULL, not the same
	if (pNode1 == NULL || pNode2 == NULL) return false;
	// check every dim, any not same, not same point
	for (int i=0; i<(int)pNode1->getData().size(); i++) {
		if (pNode1->getData().at(i) != pNode2->getData().at(i)){
			return false;
		}
	}
	return true;
}

// add start & goal and link to the map
void prmPlanner::addStartGoal(prmNode* pNode1, prmNode* pNode2) {
	int numToLink = 5000, count = 0;
	if (!nodeInCollision(pNode1)) {
		mexPrintf("start valid\n");
		start = pNode1;
		vector<prmNode*> nearest = findNN(start, numToLink);
		for (int i = 0; i<min(numToLink, (int)nearest.size()); i++){
		if (!actionInCollision(start, nearest[i])) {
				linkNode(start, nearest[i]);
			}
			else if (count==min(numToLink-1, (int)nearest.size()-1)) {
				mexPrintf("start cannot find nearest to link in %d nearest. cannot plan for a valid path\n", nearest.size());
			}
			else {count++;}
		}
	}
	else {
		mexPrintf("start not valid. cannot plan for a valid path\n");
	}
	count=0;
	if (!nodeInCollision(pNode2)) {
		mexPrintf("goal valid\n");
		goal = pNode2;
		vector<prmNode*> nearest = findNN(goal, numToLink);
		for (int i = 0; i<min(numToLink, (int)nearest.size()); i++) {
			if (!actionInCollision(goal, nearest[i])) {
				linkNode(goal, nearest[i]);
			}
			else if (count==min(numToLink-1, (int)nearest.size()-1)) {
				mexPrintf("goal cannot find nearest to link in %d nearest. cannot plan for a valid path\n", nearest.size());
			}
			else {count++;}
		}
	}
	else {
		mexPrintf("goal not valid. cannot plan for a valid path\n");
	}
}

// add edge to both nodes (undirected graph)
void prmPlanner::linkNode(prmNode* pNode1, prmNode* pNode2) {
	if (pNode1 == NULL || pNode2 == NULL) return;
	pNode1->addEdge(pNode2);
	pNode2->addEdge(pNode1);
}

// randomly generate a roadMap and save
void prmPlanner::geneRoadMap(int nNode, double minDim[], double maxDim[]) {
	int numToLink = 15;
	double radius = 0.4;
	// ensure map is clear
	clearRoadMap();
	// generate nNode prmNodes
	for (int i=0; i<nNode; i++) {
		prmNode* tmpt;
		tmpt = randNewNode(minDim, maxDim);
		// if node not in collision, add to roadMap, find NN and link 
		if (!nodeInCollision(tmpt)) {
			vector<prmNode*> nearest = findNN(tmpt, radius);
			// the nearest node should not be the same node, or drop the node
			if (nearest.size() == 0 || !isEqual(tmpt,nearest.at(0))) {
				roadMap.push_back(tmpt);
				// check if k nearest are in collision, if not, add edge
				for (int j = 0; j<(int)nearest.size(); j++) {
					if (!actionInCollision(tmpt, nearest[j])) {
						linkNode(tmpt, nearest[j]);
					}
				}
			}
		}
		// else if in collision, doesn't count to valid samples
	}
	mexPrintf("roadMap generated \n");
}

// calculate heuristic
double prmPlanner::calHeu(prmNode* pNode) {
	if (goal==NULL) return 0.0;
	return disNode(pNode, goal);
}

// plan using A*
vector<prmNode*> prmPlanner::planAStar() {
	vector<prmNode*> recordPath;
	if (start == NULL || goal == NULL) return recordPath;
	// set openList<f, prmNode*> and insert start into it
	set<pair<double, prmNode*>> openList;
	openList.insert(make_pair(calHeu(start), start));
	// unordered_map<prmNode*, <<g, if closed>, parent node>> for future back track
	unordered_map<prmNode*, pair<pair<double, bool>, prmNode*>> reachedNode;
	reachedNode[start] = make_pair(make_pair(0.0, false),start);
	int countInWhile = 0;
	// expand until openList is empty or goal reached
	while(!openList.empty()) {
		countInWhile++;
		pair<double, prmNode*> bestCell = *openList.begin();
		openList.erase(openList.begin());
		reachedNode[bestCell.second].first.second = true;
		/*----------------if goal reached ,break--------------------*/
		if (isEqual(bestCell.second, goal)) {
			mexPrintf("\n\nreach goal!!!\n\n");
			prmNode* backTrackNode = goal;
			while (1) {
				recordPath.push_back(backTrackNode);
				if (isEqual(backTrackNode, start)) {break;}
				backTrackNode = reachedNode[backTrackNode].second;
				// add interpolate points
				vector<prmNode*> middle = middleNode(recordPath.back(), backTrackNode);
				recordPath.insert(recordPath.end(),middle.begin(),middle.end()); 
			}
			break;
		}
		
		
		// for the best node, add all it's successors into openList
		for (int i = 0; i<bestCell.second->getEdge().size(); i++) {
			prmNode* tmpt = bestCell.second->getEdge().at(i);
			double newG = reachedNode[bestCell.second].first.first + disNode(bestCell.second, tmpt);
			double newF = newG + calHeu(tmpt);	
			// if not reached, add to openList
			if (reachedNode.find(tmpt) == reachedNode.end()) {
				reachedNode[tmpt] = make_pair(make_pair(newG, false),bestCell.second);
				openList.insert(make_pair(newF, tmpt));
			}
			// else if not in close list, compare and update
			else if (!reachedNode[tmpt].first.second) {
				if (newG < reachedNode[tmpt].first.first) {
					reachedNode[tmpt].first.first = newG;
					reachedNode[tmpt].second = bestCell.second;
				}
			}
			// else already in close list, do nothing
		}
	}
	mexPrintf("step:%f\n", disNode(recordPath[0], recordPath[4]));
	mexPrintf("\npathlength: %d\nsize reached: %d, in while: %d\n", recordPath.size(), reachedNode.size(), countInWhile);
	return recordPath;
}

// return the size of roadMap
int prmPlanner::getRoadMapSize() {
	return roadMap.size();
}

#endif // PRM_PLANNER