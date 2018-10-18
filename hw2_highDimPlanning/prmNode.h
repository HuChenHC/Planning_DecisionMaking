#ifndef PRM_NODE
#define PRM_NODE

#include <vector>

using namespace std;

/*-----------------------------------------
--------- a class of prm node -------------
------------------------------------------*/
class prmNode{
private:
	// angle data for several DOFs
	vector<double> data;
	// the other nodes on edges 
	vector<prmNode*> edge;
public:
	prmNode();
	prmNode(double arr[], int dim);
	prmNode(vector<double> vec);
	void setData(double arr[], int dim);
	void setData(vector<double> vec);
	void increaseData(double increment, int ind);
	vector<double> getData() {return data;}
	void addEdge(prmNode* nodeIn);
	vector<prmNode*> getEdge() {return edge;}
};

prmNode::prmNode() {}

prmNode::prmNode(double arr[], int dim) {
	setData(arr, dim);
}

prmNode::prmNode(vector<double> vec) {
	setData(vec);
}

void prmNode::setData(double arr[], int dim) {
	for (int i=0; i<dim; i++) {
		data.push_back(arr[i]);
	}
}

void prmNode::setData(vector<double> vec) {
	data = vec;
}

void prmNode::increaseData(double increment, int ind) {
	data.at(ind) = data.at(ind) + increment;
}

void prmNode::addEdge(prmNode* nodeIn) {
	edge.push_back(nodeIn);
}


#endif //PRM_NODE