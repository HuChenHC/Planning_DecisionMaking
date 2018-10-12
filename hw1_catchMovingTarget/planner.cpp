/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <set>
#include <utility>
#include <unordered_map>

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

using namespace std;

struct cellC {
	int parentX, parentY;
	double f, g, h;
};

double calHeu(int startX, int startY, int goalX, int goalY, int collision_thresh) {
	return 0.6*double(collision_thresh)*sqrt(double(pow(startX-goalX,2)+pow(startY-goalY,2)));
}

pair<int, int> backTrack(int goalposeX, int goalposeY, unordered_map<int,cellC> cellDtl, int x_size, int robotposeX, int robotposeY) {
	cellC parentCell = cellDtl[goalposeY*x_size+goalposeX];
	int parentX = parentCell.parentX, parentY = parentCell.parentY;
	int preX = goalposeX, preY = goalposeY;
	while (parentX != robotposeX || parentY != robotposeY) {
		parentCell = cellDtl[parentY*x_size+parentX];
		preX = parentX;
		preY = parentY;
		parentX = parentCell.parentX;
		parentY = parentCell.parentY;
	}
	return make_pair(preX, preY);
}

int onUntravelledTraj(int curr_time, int target_steps, double* target_traj, int robotposeX, int robotposeY) {
	for (int i=min(curr_time, target_steps-1); i<target_steps; i++) {
		if (target_traj[i] == robotposeX && target_traj[i+target_steps] == robotposeY) {
			return i;
		}
	}
	return -1;
}

static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
	// initialize action (bestX, bestY), don't move if no decision made
    int bestX = 0, bestY = 0;
	// set goal position
	int goalposeX, goalposeY;
	double robotTargetDis = sqrt(double(pow(robotposeX-targetposeX,2)+pow(robotposeY-targetposeY,2)));
	int placeOnTraj = onUntravelledTraj(curr_time, target_steps, target_traj, robotposeX, robotposeY);	
	if (robotTargetDis<30.0) {
		// if distance < 100 try to catch up
		goalposeX = (int) target_traj[curr_time];
		goalposeY = (int) target_traj[curr_time+target_steps];
	}
	else if (placeOnTraj > 0) {
		// if on untravelled trajectory and too far away to catchup, backtrack the trajectory
		// find the previous point on trajectory and return the decision
		robotposeX = target_traj[placeOnTraj-1];
		robotposeY = target_traj[placeOnTraj-1+target_steps];
		action_ptr[0] = robotposeX;
		action_ptr[1] = robotposeY;
		return;
	} 
	else {
		// else, move toward the target but a few steps forward it 
		goalposeX = (int) target_traj[curr_time+int(robotTargetDis*0.6)];
		goalposeY = (int) target_traj[curr_time+int(robotTargetDis*0.6)+target_steps];
	}
	
	/*----- implement a* algorithm   -----*/
	
	// bool closed to indicate if closed
	bool closed[x_size*y_size] = {false};

	// open list, add begin point into openlist
	// set<pair<f,pair<positionX, positionY>>>
	set<pair<double, pair<int,int>>> openList;
	openList.insert(make_pair(0.0, make_pair(robotposeX, robotposeY)));
	
	// unordered_map<index=y*x_size+x, cellC{prtX,prtY,f,g,h}> for cells that ever reached
	unordered_map<int,cellC> cellDtl;
	cellC startCell = {-1, -1, 0.0, 0.0, 0.0};
	cellDtl.insert(make_pair(robotposeY*x_size+robotposeX, startCell));
	
	int countinwhile = 0;
	while (!openList.empty()){
		// remove the cell with smallest f from open list, add it to close list
		pair<double, pair<int,int>> bestCell = *openList.begin();
		openList.erase(openList.begin());
		int x = bestCell.second.first;		// index of current best cell
		int y = bestCell.second.second;		// index of current best cell
		int ind = y*x_size+x;				// index of current best cell
		closed[ind] = true;
		if (x == goalposeX && y == goalposeY) {
			/*---------- backtrack find the successor, give it to bestX, bestY -------------*/
			pair<int, int> nextCell = backTrack(goalposeX, goalposeY, cellDtl, x_size, robotposeX, robotposeY);
			bestX = nextCell.first - robotposeX;
			bestY = nextCell.second - robotposeY;			
			break;
		}
		
		// to store f, g, h for comparasion with previous stored score 
		double newF = 0.0, newG = 0.0, newH = 0.0;
		
		// search cells around the best cell 
		//  2    4    7
		//  1   best  6
		//  0    3    5
		for (int i = 0; i<NUMOFDIRS; i++){
			int newX = x + dX[i], newY = y + dY[i];
			int newIdx = newY*x_size+newX;
			// if inside map && map < collision threshold && not in close list
			// find details and compare previous stored
			if (newX>0 && newX <= x_size && newY >0 && newY <=y_size &&
						(int)map[GETMAPINDEX(newX,newY,x_size,y_size)] < collision_thresh
						&& !closed[newIdx]) {
				newG = cellDtl[ind].g+(double)map[GETMAPINDEX(newX,newY,x_size,y_size)];
				newH = calHeu(newX, newY, goalposeX, goalposeY, collision_thresh);
				newF = newG + newH;
				
				// if the cell already in open list, compare f value & replace
				// else insert directly
				if (cellDtl.find(newIdx)!=cellDtl.end()) {
					if (newF<cellDtl[newIdx].f) {
						openList.erase(openList.find(make_pair(cellDtl[newIdx].f, make_pair(newX,newY))));
						openList.insert(make_pair(newF, make_pair(newX,newY)));
						cellDtl[newIdx].f = newF;
						cellDtl[newIdx].g = newG;
						cellDtl[newIdx].h = newH;
						cellDtl[newIdx].parentX = x;
						cellDtl[newIdx].parentY = y;
					}
				}
				else {
					openList.insert(make_pair(newF, make_pair(newX,newY)));
					cellC newCell = {x, y, newF, newG, newH};
					cellDtl.insert(make_pair(newIdx, newCell));
				}
			}
		}
		
		countinwhile++;
		if (countinwhile>MAX(70000,(75000-curr_time))) {
			pair<int, int> nextCell = backTrack(x, y, cellDtl, x_size, robotposeX, robotposeY);
			bestX = nextCell.first - robotposeX;
			bestY = nextCell.second - robotposeY;
			break;
		}
	}
	
	// publish the next step decision
    robotposeX = robotposeX + bestX;
    robotposeY = robotposeY + bestY;
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;
    return;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}