/*
*   Chen Hu
*   chu3@andrew.cmu.edu
*   Oct 9, 2018  
*/

#include <iostream> 
#include <algorithm>
#include <string.h>
#include <stddef.h>
#include <stdio.h>
#include<bits/stdc++.h> 

using namespace std; 
const int maxDim = 9;

#define PI 3.141592654

// Node: data and it's children
struct Node{
	float data[maxDim];
	Node *left, *right; 
	~Node() {
		delete left; left = NULL;
		delete right; right = NULL;
		// cout << "destructor called\n";
	}
};
// return a new node
Node* newNode(float arr[], int d);
// if two Nodes are equal
bool isEqual(Node* a, float b[], int dim);
bool isEqual(Node* a, Node* b, int dim);
// return node with smallest volumn at dimSearch, c!=NULL
Node* minInThree(Node* a, Node* b, Node* c, int dimSearch);

// BoundBox: leftBound[dim], rightBound[dim]
struct BoundBox {
	float leftBound[maxDim];
	float rightBound[maxDim];
};
// split and get left or right bound box
// left for 0 and right for 1
BoundBox splitBB(BoundBox bb, int dimSearch, float newBound,int rightBB);
// calculate the distance between a node and a bound box
float disNodeBB(float a[], BoundBox bb, int dim);
float disNodeBB(Node* a, BoundBox bb, int dim);
// calculate the distance between two nodes
float disNodeNode(float a[], float b[], int dim);


/*-----------------------------------------
----------- a class of kd tree ------------
------------------------------------------*/
class KD {
private:
	Node *root;
	Node *nearNeighbor;
	int dim;
	float bestDis;
	BoundBox bBox;
	
	// recursively insert a node
	Node* insert(Node* currRoot, float arr[], int cd);
	// recursively find nearest node
	Node* findMin(Node* currRoot, int dimSearch, int cd);
	// recursively delete given node if found
	Node* deleteNode(float arr[], Node* currRoot, int cd);
	Node* deleteNode(Node* toDelete, Node* currRoot, int cd);
	// recursively find nearest node
	Node* findNN(float arr[], Node* currRoot, int cd, BoundBox bb);
public:
	// KD constructor
	KD();
	KD(int d);
	// KD destructor
	~KD() {cleanUp();}
	// return root
	Node* getRoot();
	// reset bestDis
	void resetBestDis();
	// insert a node
	void insert(float arr[]);
	// find min at given dim
	Node* findMin(int dimSearch);
	// delete given node if found
	Node* deleteNode(Node* toDelete);
	Node* deleteNode(float arr[]);
	// find nearest node
	Node* findNN(float arr[]);
	// delete subtree of a node
	bool clean(Node* currRoot);
	// delete all nodes
	bool cleanUp();
	
};






/*----------------------------------------
*-----------------main--------------------
*-----------------------------------------
** 	here form a tree:
            (1,2,3)
                  \
                (2,3,4)
                /
            (3,1,5)
            /
        (4,2,3)
** 	test function:
KD::KD(), KD::~KD(), 
KD::insert(), KD::findMin(), KD::findNN(), 
KD::cleanUp() , KD::deleteNode()
*-----------------------------------------*/
int main() {
	KD tree(3);
	float arr[][3] = {{1,2,3}, {2,3,4}, {3,1,5}, {4,2,3},{0,0,0}};
	
	// test insert()
	tree.insert(arr[0]);
	tree.insert(arr[1]);
	tree.insert(arr[2]);
	tree.insert(arr[3]);
	cout<<"root->right->left is node: " << tree.getRoot()->right->left->data[0]<<"\n\n";
	
	// test findMin()
	Node* r = tree.findMin(1);
	cout << "min of dim 1: " << r->data[0] << "\n\n";
	
	// test deleteNode()
	Node* t = tree.deleteNode(arr[2]);
	cout << "delete the third node, now third node: " << t->right->left->data[0] << "\n\n";
	
	// test splitBB()
	BoundBox bb;
	bb = {{1,2,3},{8,8,8}};
	cout<< "test BoundBox: "<< bb.leftBound[0]<<" "<<bb.rightBound[0]<<"\n";
	cout<< "(2,3,4) distance with BoundBox: " << disNodeBB(arr[1], bb, 3)<<"\n";
	cout<< "(0,0,0) distance with BoundBox: " << disNodeBB(arr[4], bb, 3)<<"\n\n";
	
	// test findNN()
	Node* s=tree.findNN(arr[4]);
	cout<< "nearNeighbor of (0,0,0) is node: "<<s->data[0]<<"\n\n";
	
	cout<<"exit main\n";
	return 0;
}






// return a new node
Node* newNode(float arr[], int d) {
	Node* tmpt = new Node;
	for (int i=0; i<d; i++) {
		tmpt->data[i] = arr[i];
	}
	tmpt->left = NULL;
	tmpt->right = NULL;
	// cout << "node created by newNode()\n";
	return tmpt;
}
// if two Nodes are equal
bool isEqual(Node* a, float b[], int dim) {
	for (int i=0; i<dim; i++) {
		if (a->data[i]!=b[i])
			return false;
	}
	return true;
}
bool isEqual(Node* a, Node* b, int dim) {
	return isEqual(a, b->data, dim);
}
// return node with smallest volumn at dimSearch, c!=NULL
Node* minInThree(Node* a, Node* b, Node* c, int dimSearch) {
	if (a!=NULL) {
		// a, b, c !=NULL
		if (b!=NULL) {
			if (a->data[dimSearch] < b->data[dimSearch]) {
				if (a->data[dimSearch] < c->data[dimSearch]) {
					return a;
				}
				else {
					return c;
				}
			}
			else {
				if (b->data[dimSearch] < c->data[dimSearch]) {
					return b;
				}
				else {
					return c;
				}
			}
		}
		// a,c != NULL && b==NULL
		if (a->data[dimSearch] < c->data[dimSearch]) {
			return a;
		}
		else {
			return c;
		}
	}
	else {
		// b, c !=NULL && a==NULL
		if (b!=NULL) {
			if (b->data[dimSearch] < c->data[dimSearch]) {
				return b;
			}
			else {
				return c;
			}
		}
		// c!= NULL && a, b == NULL
		return c;
	}
}

// split and get left or right bound box
// left for 0 and right for 1
BoundBox splitBB(BoundBox bb, int dimSearch, float newBound,int rightBB) {
	// if dim with maxDim && newBound within BoundBox, else return old one
	if (dimSearch<maxDim && newBound>=bb.leftBound[dimSearch] && 
				newBound<bb.rightBound[dimSearch]) {
		if (rightBB) {
			bb.leftBound[dimSearch] = newBound;
			return bb;
		} 
		else {
			bb.rightBound[dimSearch] = newBound;
			return bb;
		}
	}
	return bb;
}
// calculate the distance between a node and a bound box
float disNodeBB(float a[], BoundBox bb, int dim) {
	float sum = 0.0;
	for (int i=0; i<dim; i++) {
		sum+=pow(max(max(bb.leftBound[i]-a[i], float(0.0)), a[i]-bb.rightBound[i]),2.0);
	}
	return pow(sum, 0.5);
}
float disNodeBB(Node* a, BoundBox bb, int dim) {
	return disNodeBB(a->data, bb, dim);
}
// calculate the distance between two nodes
float disNodeNode(float a[], float b[], int dim) {
	float sum = 0.0;
	for (int i=0; i<dim; i++) {
		sum+=pow(a[i]-b[i],2.0);
	}
	return pow(sum, 0.5);
}

// KD constructor
KD::KD() {
	dim = 1;
	bBox.leftBound[0] = INT_MIN;
	bBox.rightBound[0] = INT_MAX;
	bestDis = INT_MAX;
	root = NULL;
	nearNeighbor = NULL;
}
KD::KD(int d) {
	if (d<maxDim) {
		dim = d;
		for (int i=0; i<dim; i++) {
			bBox.leftBound[i] = INT_MIN;
			bBox.rightBound[i] = INT_MAX;
		}
	}
	else {
		cout<<"\n\nInvalid dim!!! tree not formed.\n\n";
	}
	bestDis = INT_MAX;
	root = NULL;
	nearNeighbor = NULL;
}
// return root
Node* KD::getRoot() {
	return root; 
}
// reset bestDis
void KD::resetBestDis() {
	bestDis = INT_MAX;
	return;
}

// recursively insert a node
Node* KD::insert(Node* currRoot, float arr[], int cd) {
	// if current node doesn't exit, create it
	if (currRoot == NULL) {
		return newNode(arr, dim);
	}
	// if duplicate node, do nothing
	if (isEqual(currRoot, arr, dim))
		return currRoot;
	// ensure current dim is within [0, dim)
	cd = cd % dim;
	// if less than node[cd], insert to left branch, else right
	if (arr[cd] < currRoot->data[cd]) {
		currRoot->left = insert(currRoot->left, arr, (cd+1)%dim);
	}
	else {
		currRoot->right = insert(currRoot->right, arr, (cd+1)%dim);
	}
	return currRoot;
}
// insert a node
void KD::insert(float arr[]) {
	root = insert(root, arr, 0);
	return;
}

// recursively find min at given dim
Node* KD::findMin(Node* currRoot, int dimSearch, int cd) {
	// empty tree || dimSearch>=dim, no min
	if (currRoot == NULL || dimSearch>=dim)
		return NULL;
	// ensure cd valid
	cd = cd % dim;
	
	// if dim searched is curr, min only in left tree
	if (dimSearch == cd) {
		if (currRoot->left == NULL) 
			return currRoot;
		return findMin(currRoot->left, dimSearch, (cd+1)%dim);
	}
	// else min would be min(currRoot, min(left), min(right))
	else {
		Node* a = findMin(currRoot->left, dimSearch, (cd+1)%dim);
		Node* b = findMin(currRoot->right, dimSearch, (cd+1)%dim);
		// find and return the min of (a, b, currRoot)
		return minInThree(a, b, currRoot, dimSearch); 
	}
}
// find min at given dim
Node* KD::findMin(int dimSearch) {
	return findMin(root, dimSearch, 0);
}

// recursively delete given node if found
Node* KD::deleteNode(float arr[], Node* currRoot, int cd) {
	// if no tree, do nothing
	if (currRoot == NULL) {
		return currRoot;
	}
	cd = cd % dim;
	// if currRoot is the node to delete
	// 1. right tree exist, replace with findMin(right)
	// 2. right tree doesn't exist, but left tree exist
	// replace with findMin(left), and swap left to right
	// 3. no left or right tree, delete this node use KD::clean()
	if (isEqual(currRoot, arr, dim)) {
		if (currRoot->right != NULL) {
			memcpy(currRoot->data, findMin(currRoot->right, cd, (cd+1)%dim)->data, sizeof(currRoot->data));
			currRoot->right = deleteNode(currRoot->data, currRoot->right, (cd+1)%dim);
		}
		else if (currRoot->left != NULL) {
			memcpy(currRoot->data, findMin(currRoot->left, cd, (cd+1)%dim)->data, sizeof(currRoot->data));
			currRoot->right = deleteNode(currRoot->data, currRoot->left, (cd+1)%dim);
			currRoot->left = NULL;
		}
		else {
			clean(currRoot);
		}
	}	// else, turn left or right to find the node and delete
	else {
		if (arr[cd] < currRoot->data[cd]) {
			deleteNode(arr, currRoot->left, (cd+1)%dim);
		}
		else {
			deleteNode(arr, currRoot->right, (cd+1)%dim);
		}
	}
	return currRoot;
}
Node* KD::deleteNode(Node* toDelete, Node* currRoot, int cd) {
	return deleteNode(toDelete->data, currRoot, cd);
}
Node* KD::deleteNode(Node* toDelete) {
	return deleteNode(toDelete->data, root, 0);
}
Node* KD::deleteNode(float arr[]) {
	return deleteNode(arr, root, 0);
}

// find nearest node
Node* KD::findNN(float arr[], Node* currRoot, int cd, BoundBox bb) {
	// if BoundBox too far, or if no tree, do nothing
	if (currRoot == NULL || disNodeBB(arr, bb, dim) > bestDis) {
		return nearNeighbor;
	}
	// if currRoot better than best, update bestDis & nearNeighbor
	float temp = disNodeNode(arr, currRoot->data, dim);
	if (temp < bestDis) {
		bestDis = temp;
		memcpy(nearNeighbor->data, currRoot->data, sizeof(currRoot->data));
	}
	// else search two subBoundBox in most promising order
	cd = cd%dim;
	if (arr[cd]<currRoot->data[cd]) {
		findNN(arr, currRoot->left, (cd+1)%dim, splitBB(bb, cd, currRoot->data[cd], 0));
		findNN(arr, currRoot->left, (cd+1)%dim, splitBB(bb, cd, currRoot->data[cd], 1));
	}
	else {
		findNN(arr, currRoot->left, (cd+1)%dim, splitBB(bb, cd, currRoot->data[cd], 1));
		findNN(arr, currRoot->left, (cd+1)%dim, splitBB(bb, cd, currRoot->data[cd], 0));
	}
	return nearNeighbor;
}
Node* KD::findNN(float arr[]) {
	if (root!=NULL) {
		nearNeighbor = newNode(root->data, dim);
		bestDis = disNodeNode(arr, nearNeighbor->data, dim);
	}
	return findNN(arr, root, 0, bBox);
}

// delete subtree of a node
bool KD::clean(Node* currRoot) {
	// if current root doesn't exist, do nothing
	if (currRoot == NULL) {
		return true;
	}
	// else if it has no children, delete current node
	else if (currRoot->left == NULL && currRoot->right == NULL) {
		delete currRoot;
		currRoot = NULL;
		return true;
	}
	// else, recursively delete it's children
	else {
		if (!clean(currRoot->left)) {
			return false;
		}
		if (!clean(currRoot->right)) {
			return false;
		}
	}
	// after children cleared, delete itself
	delete currRoot;
	currRoot = NULL;
	return true;
}

// delete all nodes
bool KD::cleanUp() {
	// delete the subtree of root
	if (!clean(root))
		return false;
	return false;
}
