#ifndef HELPER_FUNC
#define HELPER_FUNC


#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
// #include <vector>
// #include <cfloat>
// #include <iostream> 
// #include <math.h>
// #include <set>
// #include <utility>
// #include <unordered_map>
// #include "mex.h"
// #include "prmNode.h"

#if !defined(GETMAPINDEX)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)
#endif

#if !defined(LINKLENGTH_CELLS)
#define LINKLENGTH_CELLS 10
#endif

#if !defined(PI)
#define PI 3.141592654
#endif


using namespace std;



// return a random double between fMin and fMax
double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

// check if a line is in collision
bool lineMapCollision(double x0, double y0, double x1, double y1,
			double*	map, int x_size, int y_size)
{
	//make sure line segment inside the environment
	if(x0 < 0 || x0 >= x_size || x1 < 0 || x1 >= x_size ||
			y0 < 0 || y0 >= y_size || y1 < 0 || y1 >= y_size) {
		return 1;
	}
	
	for (int i=0; i<LINKLENGTH_CELLS; i++) {
		// check 10 points from line, if any point in collision, line in collision
		int nX = round(x0 + i * (x1-x0)/(LINKLENGTH_CELLS));
		int nY = round(y0 + i * (y1-y0)/(LINKLENGTH_CELLS));
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1) {
			return true;
		}
	} 	
	if(map[GETMAPINDEX((int)x1,(int)y1,x_size,y_size)] == 1) {
		return true;
	}
	return false;
}


// check if node in collision
bool nodeCol(double* rNode, double* map, int xSize, int ySize, int dim){
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
		x1 = x0 + LINKLENGTH_CELLS*cos(2.0*PI-*(rNode+i));
		y1 = y0 - LINKLENGTH_CELLS*sin(2.0*PI-*(rNode+i));
		//check the validity of the corresponding line segment
		if(lineMapCollision(x0,y0,x1,y1,map,xSize,ySize)) {
			return true;
		}
	}    
    return false; 	
}


#endif // HELPER_FUNC
