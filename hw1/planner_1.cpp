/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <mex.h>

// #include "a_star.h"
#include "forward_dijkstra.h"

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

    int time_cutoff = 100;

    static std::vector<std::pair<int, int>> best_path;
    int next_x, next_y;
    int distance_threshold = 100; // maybe change this parameter to map size dependent

    static GlobalDijkstraPlanner global_dijkstra;
    static int counter = 0;

    if (!global_dijkstra.isInitialized()) {
        // Initialize the planner at the very beginning (at timestamp 0)

        // load the map into planner (this will be done only once)
        mexPrintf("1111\n");
        global_dijkstra.initMapSize(x_size, y_size);
        mexPrintf("2222\n");
        global_dijkstra.initParam(std::make_pair(robot_x, robot_y), std::make_pair(target_x, target_y), collision_thresh);
        mexPrintf("3333\n");
        for (int i=1; i<=x_size; ++i) {
            if (i % 100 == 0)
                mexPrintf("Map loaded: %d rows\n"   , i);
            for (int j=1; j<=y_size; ++j) {
                int cost = map[GETMAPINDEX(i, j, x_size, y_size)];
                global_dijkstra.map[i-1][j-1] = cost;
                // global_dijkstra.cost_map[i-1][j-1] = cost;
                // mexPrintf("111111\n");
                // global_dijkstra.initMapCell(std::make_pair(i-1, j-1), cost);
                // mexPrintf("22222\n");
                // mexPrintf("Cost: %d \n", cost);
            }
        }

        std::vector<std::pair<int, int>> target_trajectory;
        mexPrintf("Target steps: %d\n", target_steps);
        for (int i=0; i<target_steps; ++i) {
            int x = (int) target_traj[i] - 1;
            int y = (int) target_traj[i + target_steps] - 1;
            target_trajectory.push_back(std::make_pair(x, y));
        }

        global_dijkstra.initTargetTrajectory(target_trajectory);
        mexPrintf("Target traj size: %d\n", global_dijkstra.target_trajectory.size());
        mexPrintf("Num of cells: %d\n", x_size * y_size);

        mexPrintf("Planner is initialized.\n");

        const auto robot_pose = std::make_pair(robot_x, robot_y);
        const auto target_pose = std::make_pair(target_x, target_y);

        // Expand the entire map from current location
        global_dijkstra.init(robot_pose);
        while (!global_dijkstra.isCompleted()) {
            int num = global_dijkstra.closed_list.size();
            if (num % 1000 == 0)
                mexPrintf("Dijkstra explored: %d\n", global_dijkstra.closed_list.size());
            
            // mexPrintf("0000\n");
            auto coordinate_cell = global_dijkstra.getCoordinateCellPairWithSmallestF();
            // mexPrintf("1111\n");
            auto curr_coordinate = coordinate_cell.first;
            // mexPrintf("2222\n");
            if (global_dijkstra.isInClosedList(curr_coordinate)) {
                continue;
            }
            auto curr_cell = coordinate_cell.second;
            global_dijkstra.addCellToClosedList(curr_coordinate, curr_cell);
            // mexPrintf("3333\n");
            int curr_x = curr_coordinate.first;
            int curr_y = curr_coordinate.second;

            // if(curr_cell.step_distance > time_cutoff)
            //     break;
            // int curr_step_distance = curr_cell.step_distance;
            // if (curr_step_distance > distance_threshold) {
            //     const auto& path = global_dijkstra.findPath(curr_coordinate);
                    
            //     next_x = path.path.back().first;
            //     next_y = path.path.back().second;
            //     action_ptr[0] = next_x + 1;
            //     action_ptr[1] = next_y + 1;
            //     return;
            // }

            // expand the surrounding cells (8-connected grid)
            for (int i=0; i<NUMOFDIRS; ++i) {
                // mexPrintf("Expanding1...\n");
                int new_x = curr_x + dX[i];
                int new_y = curr_y + dY[i];
                // mexPrintf("4444\n");
                const auto new_coordinate = std::make_pair(new_x, new_y);
                if (global_dijkstra.isValidCoordinate(new_coordinate) && !global_dijkstra.isInClosedList(new_coordinate)) {
                    // mexPrintf("Expanding2...\n");
                    auto new_cell = global_dijkstra.constructCell(new_coordinate, curr_x, curr_y, curr_cell.step_distance+1);
                    // mexPrintf("5555\n");
                    // if (global_dijkstra.isInOpenList(new_coordinate)) {
                    //     // auto old_cell = global_dijkstra.getCellFromOpenList(new_coordinate);
                    //     // mexPrintf("6666\n");
                    //     const bool lower_cost = new_cell.cost < global_dijkstra.cost_map[new_x][new_y];
                    //     if (lower_cost) {
                    //         // mexPrintf("7777\n");
                    //         global_dijkstra.updateCell(new_coordinate, curr_coordinate.first, curr_coordinate.second, new_cell.cost, curr_cell.step_distance+1);
                    //         global_dijkstra.cost_map[new_x][new_y] = new_cell.cost;
                    //         global_dijkstra.cost_to_coordinate.push(GlobalDijkstraPlanner::CostCoordinate(new_cell.cost, new_coordinate));
                    //     }
                    // }

                    // else {
                    //     // mexPrintf("8888\n");
                    //     global_dijkstra.addCellToOpenList(new_coordinate, new_cell);
                        
                    // }
                    if (global_dijkstra.isInOpenList(new_coordinate)) {
                        // mexPrintf("6666\n");
                        const bool lower_cost = new_cell.cost < global_dijkstra.cost_map[new_x][new_y];
                        if (lower_cost) {
                            // mexPrintf("7777\n");
                            // global_dijkstra.updateCell(new_coordinate, curr_coordinate.first, curr_coordinate.second, new_cell.cost, curr_cell.step_distance+1);
                            // global_dijkstra.cost_map[new_x][new_y] = new_cell.cost;
                            global_dijkstra.parents[new_x][new_y].first = curr_x;
                            global_dijkstra.parents[new_x][new_y].second = curr_y;
                            global_dijkstra.addCellToOpenList(new_coordinate, new_cell);
                            // global_dijkstra.cost_to_coordinate.push(GlobalDijkstraPlanner::CostCoordinate(new_cell.cost, new_coordinate));
                        }
                    }
                    else {
                        // mexPrintf("8888\n");
                        // global_dijkstra.cost_map[new_x][new_y] = new_cell.cost;
                        global_dijkstra.parents[new_x][new_y].first = curr_x;
                        global_dijkstra.parents[new_x][new_y].second = curr_y;
                        global_dijkstra.addCellToOpenList(new_coordinate, new_cell);
                        // global_dijkstra.cost_to_coordinate.push(GlobalDijkstraPlanner::CostCoordinate(new_cell.cost, new_coordinate));

                    }
                }
            }


        }
        mexPrintf("9999\n");
        // process points on target trajectory
        int target_traj_counter = 0;
        for (const auto& point : global_dijkstra.target_trajectory) {
            if (target_traj_counter % 100 == 0) {
                mexPrintf("Processed %d points on trajectory\n", target_traj_counter);
            }
            target_traj_counter++;
            int target_arrival_time = point.second;
            auto coordinate = point.first;
            if (target_arrival_time > curr_time) {
                // mexPrintf("0000\n");
                // backtrace to find the path
                auto path = global_dijkstra.findPath(coordinate);
                int wait_time = target_arrival_time - curr_time - path.length;
                if (wait_time > 1) {
                    global_dijkstra.evaluated_paths.insert({path.cost + wait_time*global_dijkstra.map[coordinate.first][coordinate.second], path});
                }
            }

        }

        best_path = global_dijkstra.evaluated_paths.begin()->second.path;

        // for (const auto& p : best) {
        //     best_path = 
        // }
    }

    // run the planner for current start and goal


    // const auto& path = global_dijkstra.findPath(robot_pose);
    // const auto& path = global_dijkstra.findBestPath(robot_pose, curr_time);
    // if (path.length > path.position_on_target_traj - curr_time) {
    //     // then we are not gonna be there on time, before target passes that goal point
    //     // on its trajectory
    //     mexPrintf("No! \n");
    // }
    // next_x = path.path[0].first;
    // next_y = path.path[0].second;
    mexPrintf("path size: %d\n", best_path.size());
    mexPrintf("counter: %d\n", counter);
    if (counter >= best_path.size() - 1) {
        next_x = best_path[0].first;
        next_y = best_path[0].second;
    }
    else {
        next_x = best_path[best_path.size() - counter - 1].first;
        next_y = best_path[best_path.size() - counter - 1].second;
    }
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