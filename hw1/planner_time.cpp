/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <mex.h>

#include "a_star.h"
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

    static AStarPlanner astar_planner;
    static GlobalDijkstraPlanner global_dijkstra;

    if (!astar_planner.isInitialized()) {
        // Initialize the planner at the very beginning (at timestamp 0)

        // load the map into planner (this will be done only once)
        astar_planner.initMapSize(x_size, y_size);
        global_dijkstra.initMapSize(x_size, y_size);

        astar_planner.initParam(std::make_pair(robot_x, robot_y), std::make_pair(target_x, target_y), collision_thresh);
        global_dijkstra.initParam(std::make_pair(robot_x, robot_y), std::make_pair(target_x, target_y), collision_thresh);

        for (int i=1; i<=x_size; ++i) {
            // if (i % 10 == 0)
            //     mexPrintf("Map loaded: %d rows\n", i);
            for (int j=1; j<=y_size; ++j) {
                int cost = map[GETMAPINDEX(i, j, x_size, y_size)];
                astar_planner.initMapCell(std::make_pair(i-1, j-1), cost);
                global_dijkstra.initMapCell(std::make_pair(i-1, j-1), cost);
                // mexPrintf("Cost: %d \n", cost);
            }
        }

        std::vector<std::pair<int, int>> target_trajectory;
        mexPrintf("Target steps: %d\n", target_steps);
        for (int i=0; i<target_steps; ++i) {
            int x = target_traj[i];
            int y = target_traj[i + target_steps];
            target_trajectory.push_back(std::make_pair(x, y));
        }
        astar_planner.initTargetTrajectory(target_trajectory);
        global_dijkstra.initTargetTrajectory(target_trajectory);

        mexPrintf("Target traj size: %d\n", astar_planner.target_trajectory.size());

        mexPrintf("Num of cells: %d\n", x_size * y_size);

        mexPrintf("Planner is initialized.\n");
        global_dijkstra.init();
        while (!global_dijkstra.isCompleted()) {
            // int num = global_dijkstra.closed_list.size();
            // if (num % 10000 == 0)
            //     mexPrintf("Dijkstra explored: %d\n", global_dijkstra.closed_list.size());
            
            auto coordinate_cell = global_dijkstra.getCoordinateCellPairWithSmallestF();
            auto curr_coordinate = coordinate_cell.first;
            auto curr_cell = coordinate_cell.second;
            global_dijkstra.addCellToClosedList(curr_coordinate, curr_cell);

            int curr_x = curr_coordinate.first;
            int curr_y = curr_coordinate.second;

            // expand the surrounding cells (8-connected grid)
            for (int i=0; i<NUMOFDIRS; ++i) {
                int new_x = curr_x + dX[i];
                int new_y = curr_y + dY[i];

                const auto new_coordinate = std::make_pair(new_x, new_y);
                if (global_dijkstra.isValidCoordinate(new_coordinate) && !global_dijkstra.isInClosedList(new_coordinate)) {

                    auto new_cell = global_dijkstra.constructCell(new_coordinate, curr_x, curr_y);

                    if (global_dijkstra.isInOpenList(new_coordinate)) {
                        auto old_cell = global_dijkstra.getCellFromOpenList(new_coordinate);

                        const bool lower_cost = new_cell.cost < old_cell.cost;
                        if (lower_cost) {
                            global_dijkstra.updateCell(new_coordinate, curr_coordinate.first, curr_coordinate.second, new_cell.cost);
                        }
                    }

                    else {
                        global_dijkstra.addCellToOpenList(new_coordinate, new_cell);
                        
                    }
                }
            }
        }
    }

    // Above only runs once
    // Below runs once every loop

    const auto robot_pose = std::make_pair(robot_x, robot_y);
    const auto target_pose = std::make_pair(target_x, target_y);
    // run the planner for current start and goal
    int next_x, next_y;
    int distance_threshold = 100; // maybe change this parameter to map size dependent

    const auto& path = global_dijkstra.findPath(robot_pose);
    // const auto& path = global_dijkstra.findBestPath(robot_pose, curr_time);
    if (path.length > path.position_on_target_traj - curr_time) {
        // then we are not gonna be there on time, before target passes that goal point
        // on its trajectory
        mexPrintf("No! \n");
    }
    next_x = path.path[0].first;
    next_y = path.path[0].second;

    // if (!astar_planner.best_goal.empty()) {
    //     target_x = astar_planner.best_goal.begin()->second.first;
    //     target_y = astar_planner.best_goal.begin()->second.second;
    // }

    // astar_planner.init(robot_x, robot_y, target_x, target_y);
    // for (const auto& pair : global_dijkstra.closed_list) {
    //     astar_planner.heuristics.insert({pair.first, pair.second.cost});
    // }

    // // if (astar_planner.isCompleted()) {
    // //     // mexPrintf("No! \n");
    // // }

    // // if (astar_planner.isOnTargetTrajectory(std::make_pair(robot_x, robot_y))) {
    // //     // mexPrintf("STAY! \n");
    // //     next_x = robot_x;
    // //     next_y = robot_y;
    // //     action_ptr[0] = next_x + 1;
    // //     action_ptr[1] = next_y + 1;
    
    // //     return;
    // // }

    // while (!astar_planner.isCompleted()) {
    //     // mexPrintf("Openlist %d \n", astar_planner.open_list.size());
    //     auto coordinate_cell = astar_planner.getCoordinateCellPairWithSmallestF();
    //     auto curr_coordinate = coordinate_cell.first;
    //     auto curr_cell = coordinate_cell.second;
    //     astar_planner.addCellToClosedList(curr_coordinate, curr_cell);

    //     int position_on_target_traj = astar_planner.isOnTargetTrajectory(curr_coordinate);
    //     if (position_on_target_traj != -1) {
    //         mexPrintf("On trajectory! \n");
    //         const auto& path = astar_planner.findPath(curr_coordinate);
    //         int wait_time = position_on_target_traj - path.length - curr_time;
    //         if (wait_time > 0) {
    //             int actual_cost = curr_cell.G + wait_time * curr_cell.G;
    //             astar_planner.best_goal.insert({actual_cost, curr_coordinate});
    //         }

    //     }

    //     // mexPrintf("Size of closed list: %d \n", astar_planner.closed_list.size());
    //     // check whether this is the goal cell?
    //     int curr_x = curr_coordinate.first;
    //     int curr_y = curr_coordinate.second;

    //     if (curr_x == target_x && curr_y == target_y) {
    //         // goal found
    //         mexPrintf("Goal found! \n");
    //         auto next_coordinate = astar_planner.getNextAction(std::make_pair(target_x, target_y));
    //         // mexPrintf("Next action got! \n");
    //         next_x = next_coordinate.first;
    //         next_y = next_coordinate.second;

    //         // next_x = curr_x;
    //         // next_y = curr_y;
    //         break;
    //     }

    //     // terminate if we searched too much
    //     if (abs(robot_x - curr_x) + abs(robot_y - curr_y) >= distance_threshold) {
    //         mexPrintf("Out of bound! \n");
    //         mexPrintf("Open list size: %d \n", astar_planner.open_list.size());
    //         auto next_coordinate = astar_planner.getNextAction(std::make_pair(curr_x, curr_y));
    //         // mexPrintf("Next action got! \n");
    //         next_x = next_coordinate.first;
    //         next_y = next_coordinate.second;
    //         break;
    //     }

    //     // expand the surrounding cells (8-connected grid)
    //     for (int i=0; i<NUMOFDIRS; ++i) {
    //         int new_x = curr_x + dX[i];
    //         int new_y = curr_y + dY[i];

    //         const auto new_coordinate = std::make_pair(new_x, new_y);
    //         if (astar_planner.isValidCoordinate(new_coordinate) && !astar_planner.isInClosedList(new_coordinate)) {

    //             auto new_cell = astar_planner.constructCell(new_coordinate, curr_x, curr_y);
    //             // mexPrintf("Cost: %d \n", new_cell.G);
    //             // mexPrintf("Heuristic: %d \n", new_cell.H);

    //             if (astar_planner.isInOpenList(new_coordinate)) {
    //                 auto old_cell = astar_planner.getCellFromOpenList(new_coordinate);

    //                 const bool lower_cost = new_cell.G < old_cell.G;
    //                 if (lower_cost) {
    //                     astar_planner.updateCell(new_coordinate, curr_coordinate.first, curr_coordinate.second, new_cell.G);
    //                 }
    //             }

    //             else {
    //                 astar_planner.addCellToOpenList(new_coordinate, new_cell);
    //                 // mexPrintf("Add (%d, %d)! \n", new_coordinate.first, new_coordinate.second);
    //             }
    //             // check if this cell is already in open list
    //                 // if yes, check for lower cost
    //                     // if the new cost is lower, modify the previous cell in the open list:
    //                         // update its g value
    //                         // update the parent
    //                         // update its f value
    //                     // else continue
    //                 // if not, add a new cell into the open list
    //         }

    //         else {

    //         }
    //     }
    // }

    // int goalposeX = (int) target_traj[target_steps-1];
    // int goalposeY = (int) target_traj[target_steps-1+target_steps];
    // // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // // printf("goal: %d %d;\n", goalposeX, goalposeY);
    // // printf("current time: %d;\n", curr_time);

    // int bestX = 0, bestY = 0; // robot will not move if greedy action leads to collision
    // double olddisttotarget = (double)sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX) + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
    // double disttotarget;
    // for(int dir = 0; dir < NUMOFDIRS; dir++)
    // {
    //     int newx = robotposeX + dX[dir];
    //     int newy = robotposeY + dY[dir];

    //     if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
    //     {
    //         if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
    //         {
    //             disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
    //             if(disttotarget < olddisttotarget)
    //             {
    //                 olddisttotarget = disttotarget;
    //                 bestX = dX[dir];
    //                 bestY = dY[dir];
    //             }
    //         }
    //     }
    // }
    // robotposeX = robotposeX + bestX;
    // robotposeY = robotposeY + bestY;
    // action_ptr[0] = robotposeX;
    // action_ptr[1] = robotposeY;

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