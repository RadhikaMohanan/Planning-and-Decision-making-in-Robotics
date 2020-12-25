/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <mex.h>
#include "Dijkstra.h" // Dijkstra header file

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
    
    int robotx = robotposeX - 1;
    int roboty = robotposeY - 1;
    int targetx = targetposeX - 1;
    int targety = targetposeY - 1;
    
    static Dijkstra Dijkstra;
    static Dijkstra::Path optimal_path;
    //Loading Map
    if (!Dijkstra.isInitialized()) {
        mexPrintf("Number of cells: %d\n", x_size * y_size);
        Dijkstra.initmap(x_size, y_size);
        for (int i=1; i<=x_size; ++i) {
            for (int j=1; j<=y_size; ++j) {
                int cost = (int) map[GETMAPINDEX(i, j, x_size, y_size)];
                Dijkstra.map[i-1][j-1] = cost;
            }
        }
     //Loading Trajectory
        std::vector<std::pair<int, int>> target_trajectory;
        mexPrintf("Target steps: %d\n", target_steps);
        for (int i=0; i<target_steps; ++i) {
            int x = (int) target_traj[i] - 1;
            int y = (int) target_traj[i + target_steps] - 1;
            target_trajectory.push_back(std::make_pair(x, y));
        }
        Dijkstra.init_target_traj(target_trajectory);
        mexPrintf("Target trajectory size: %d\n", Dijkstra.target_trajectory.size());
        Dijkstra.init(robotx, roboty, collision_thresh);

        Dijkstra.search();
        
        optimal_path = Dijkstra.evaluated_paths.begin()->second;
       
    }

    
    static int count = 0;
    int init_plan_time = 0;
    if (curr_time - count > 1) {
        init_plan_time = curr_time - count;
    }

    auto optimal_trajectory = optimal_path.trajectory;

    static bool already_trimmed = false;

    if (init_plan_time > 0 && !already_trimmed) {
        mexPrintf("Initial planning time: %d\n", init_plan_time);
        auto itr = Dijkstra.evaluated_paths.begin();
        while (itr->second.wait_time <= init_plan_time) {
            itr++;
            mexPrintf("Path changed\n");
        }
        optimal_path = itr->second;
        mexPrintf("Initial planning takes too long, reducing waiting time and trimming path...\n");

        mexPrintf("Path size before trimming: %d\n", optimal_path.trajectory.size());
        int trimmed_time = init_plan_time;
        if (optimal_path.wait_time >= init_plan_time) {
            optimal_path.trajectory.erase(optimal_path.trajectory.begin() + optimal_path.least_cost_index, optimal_path.trajectory.begin() + optimal_path.least_cost_index + trimmed_time);
        }
        mexPrintf("Path size after trimming: %d\n", optimal_path.trajectory.size());

        optimal_trajectory = optimal_path.trajectory;
        already_trimmed = true;
    }
    
    int next_x, next_y;

    int path_size = optimal_trajectory.size();

    int i = std::max(0, path_size - count - 1);

    next_x = optimal_trajectory[i].first;
    next_y = optimal_trajectory[i].second;
 
    count++;

    // convert to 1-indexed
    action_ptr[0] = next_x + 1;
    action_ptr[1] = next_y + 1;

    
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