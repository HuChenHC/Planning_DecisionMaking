#ifndef RRT_NODE
#define RRT_NODE

#include <vector>
#include <cfloat>

using namespace std;

/*-----------------------------------------
--------- a class of rrt node -------------
------------------------------------------*/
class rrtNode{
protected:
	// angle data for several DOFs
	vector<double> data;
	// parent of node
	rrtNode* parent;
	// g value for rrt star
	double gValue;
public:
	rrtNode();
	rrtNode(double arr[], int dim);
	rrtNode(vector<double> vec);
	void setData(double arr[], int dim);
	void setData(vector<double> vec);
	void increaseData(double increment, int ind);
	vector<double> getData() {return data;}
	void setParent(rrtNode* rNode);
	rrtNode* getParent() {return parent;}
	double getGValue();
	void setGValue(double a);
};

rrtNode::rrtNode() {}

rrtNode::rrtNode(double arr[], int dim) {
	setData(arr, dim);
	gValue = DBL_MAX;
}

rrtNode::rrtNode(vector<double> vec) {
	setData(vec);
	gValue = DBL_MAX;
}

void rrtNode::setData(double arr[], int dim) {
	for (int i=0; i<dim; i++) {
		data.push_back(arr[i]);
	}
}

void rrtNode::setData(vector<double> vec) {
	data = vec;
}

void rrtNode::increaseData(double increment, int ind) {
	data.at(ind) = data.at(ind) + increment;
}


void rrtNode::setParent(rrtNode* rNode) {
	parent = rNode;
}

double rrtNode::getGValue() {
	return gValue;
}

void rrtNode::setGValue(double a) {
	gValue = a;
}
#endif  // RRT_NODE