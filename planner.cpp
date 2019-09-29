/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <mex.h>

// #include "a_star.h"
#include "dijkstra.h"

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

    // note that the robotpose and targetpose here are 1-indexed
    int robot_x = robotposeX - 1;
    int robot_y = robotposeY - 1;
    int target_x = targetposeX - 1;
    int target_y = targetposeY - 1;

    static Dijkstra dijkstra;
    static Dijkstra::Path best_path;
    // static std::vector<std::pair<int, int>> best_path;

    if (!dijkstra.isInitialized()) {
        // run this for once only, in the very beginning

        // load the map
        mexPrintf("Num of cells: %d\n", x_size * y_size);
        dijkstra.initMapSize(x_size, y_size);
        for (int i=1; i<=x_size; ++i) {
            for (int j=1; j<=y_size; ++j) {
                int cost = (int) map[GETMAPINDEX(i, j, x_size, y_size)];
                dijkstra.map[i-1][j-1] = cost;
            }
        }

        // load the targe trajectory
        std::vector<std::pair<int, int>> target_trajectory;
        // mexPrintf("Target steps: %d\n", target_steps);
        for (int i=0; i<target_steps; ++i) {
            int x = (int) target_traj[i] - 1;
            int y = (int) target_traj[i + target_steps] - 1;
            target_trajectory.push_back(std::make_pair(x, y));
        }
        // mexPrintf("Trajectory size: %d\n", target_trajectory.size());

        dijkstra.initTargetTrajectory(target_trajectory);
        mexPrintf("Target trajectory size: %d\n", dijkstra.target_trajectory.size());
        
        mexPrintf("Planner is initialized.\n");

        dijkstra.init(robot_x, robot_y, collision_thresh);

        dijkstra.search();

        // mexPrintf("Evaluated path size: %d\n", dijkstra.evaluated_paths.size());

        best_path = dijkstra.evaluated_paths.begin()->second;
        // for (const auto& p : best_path.trajectory) {
        //     mexPrintf("Coordinate: %d, %d\n", p.first, p.second);
        // }
        mexPrintf("Path cost: %d\n", best_path.cost); 
        mexPrintf("Waiting time: %d\n", best_path.wait_time);
        // mexPrintf("Index: %d\n", best_path.least_cost_index); 
        mexPrintf("Waiting point: (%d, %d)\n", best_path.least_cost_coordinate.first, best_path.least_cost_coordinate.second); 

        mexPrintf("Starting point cost: %d\n", dijkstra.map[robot_x][robot_y]);
        mexPrintf("Waiting point cost: %d\n", dijkstra.map[best_path.least_cost_coordinate.first][best_path.least_cost_coordinate.second]);
        // mexPrintf("Least cost: %d\n", dijkstra.map[709][574]);
        // mexPrintf("Least cost: %d\n", dijkstra.map[693][575]); 
        // auto pair = std::make_pair(709, 574);
        // mexPrintf("Target arrival time: %d\n", dijkstra.target_trajectory.find(pair)->second);
    }

    auto best_trajectory = best_path.trajectory;

    // mexPrintf("Current time: %d\n", curr_time);
    static int counter = 0;
    int initial_plan_time = 0;
    if (curr_time - counter > 1) {
        initial_plan_time = curr_time - counter;
    }

    static bool already_trimmed = false;

    if (initial_plan_time > 0 && !already_trimmed) {
        mexPrintf("Initial planning time: %d\n", initial_plan_time);
        auto itr = dijkstra.evaluated_paths.begin();
        while (itr->second.wait_time <= initial_plan_time) {
            itr++;
            mexPrintf("Path changed\n");
        }
        best_path = itr->second;
        // for (const auto& p : best_path.trajectory) {
        //     mexPrintf("Coordinate: %d, %d\n", p.first, p.second);
        // }
        mexPrintf("Initial planning takes too long, shrinking waiting time and trimming path...\n"

        mexPrintf("Path size before trimming: %d\n", best_path.trajectory.size());
        int trimmed_time = initial_plan_time;
        if (best_path.wait_time >= initial_plan_time) {
            best_path.trajectory.erase(best_path.trajectory.begin() + best_path.least_cost_index, best_path.trajectory.begin() + best_path.least_cost_index + trimmed_time);
        }
        // for (const auto& p : best_path.trajectory) {
        //     mexPrintf("Coordinate: %d, %d\n", p.first, p.second);
        // }
        mexPrintf("Path size after trimming: %d\n", best_path.trajectory.size());

        best_trajectory = best_path.trajectory;
        already_trimmed = true;
    }
    

    int next_x, next_y;

    int path_size = best_trajectory.size();

    // mexPrintf("Best path size: %d\n", path_size);
    // if (counter >= path_size) {
    //     next_x = best_path[0].first;
    //     next_y = best_path[0].second;
    // } else {
    //     next_x = best_path[path_size - counter - 1].first;
    //     next_y = best_path[path_size - counter - 1].second;
    // }
    int i = std::max(0, path_size - counter - 1);

    next_x = best_trajectory[i].first;
    next_y = best_trajectory[i].second;
    // mexPrintf("Coordinate: %d, %d\n", next_x, next_y);
    // mexPrintf("Coordinate: %d, %d\n", best_path[0].first, best_path[0].second);
    counter++;

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