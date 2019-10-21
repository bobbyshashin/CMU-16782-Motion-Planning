/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include "RRT_vector.h"
#include "RRT_Connect.h"
#include "RRT_Star.h"
#include "PRM.h"

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm (should be the same as the one used in runtest.m)
#define LINKLENGTH_CELLS 10

static void planner(
           int planner_id,
		   double*	map,
		   int x_size,
 		   int y_size,
           double* armstart_anglesV_rad,
           double* armgoal_anglesV_rad,
	   int numofDOFs,
	   double*** plan,
	   int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
    
    // placeholder for now, unused
    double* joint_limits;

    int K = 20000;
    double eps = 0.5;
    double reached_threshold = 0.5;
    double goal_bias_weight = 0.1;
    double radius = 1.0;
    int max_successors = 5;
    std::vector<std::vector<double>> path;

    if (planner_id == RRT) {
        RRT_Planner rrt(K, eps, reached_threshold, numofDOFs, joint_limits, armstart_anglesV_rad, armgoal_anglesV_rad, x_size, y_size, map, LINKLENGTH_CELLS, PI, goal_bias_weight);
        rrt.init();
        rrt.growTree();
        if (rrt.goal_reached) {
            path = rrt.findPath();
        }
    } else if (planner_id == RRTCONNECT) {
        RRT_Connect_Planner rrt_connect(K, eps, reached_threshold, numofDOFs, joint_limits, armstart_anglesV_rad, armgoal_anglesV_rad, x_size, y_size, map, LINKLENGTH_CELLS, PI, goal_bias_weight);
        rrt_connect.init();
        rrt_connect.growTree();

        std::vector<std::vector<double>> orig_path;
        if (rrt_connect.goal_reached) {
            orig_path = rrt_connect.findPath();
        }
        mexPrintf("Path size: %d \n", orig_path.size());
        path = rrt_connect.interpolatePath(orig_path);
        mexPrintf("Interpolated path size: %d \n", path.size());


    } else if (planner_id == RRTSTAR) {
        RRT_Star_Planner rrt_star(K, eps, reached_threshold, radius, numofDOFs, joint_limits, armstart_anglesV_rad, armgoal_anglesV_rad, x_size, y_size, map, LINKLENGTH_CELLS, PI, goal_bias_weight);
        rrt_star.init();
        rrt_star.growTree();
        if (rrt_star.goal_reached) {
            path = rrt_star.findPath();
        }
    } else if (planner_id == PRM) {
        PRM_Planner prm(K, max_successors, reached_threshold, radius, numofDOFs, joint_limits, armstart_anglesV_rad, armgoal_anglesV_rad, x_size, y_size, map, LINKLENGTH_CELLS, PI, goal_bias_weight);
        prm.init();
        prm.buildRoadmap();
    }



    if (!path.empty()) {
        // for (int i=1; i<path.size(); ++i) {
        //     mexPrintf("distance: %f\n", rrt.euclideanDist(path[i-1], path[i]));
        // }

        *planlength = path.size();

        *plan = (double**) malloc(path.size()*sizeof(double*));
        for (int i=0; i<path.size(); ++i) {
            (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
            for(int j = 0; j < numofDOFs; ++j){
                (*plan)[i][j] = path[i][j];
            }
        }
    }
    // if (false) {}
    else {
        // for now just do straight interpolation between start and goal checking for the validity of samples

        double distance = 0;
        int i,j;
        for (j = 0; j < numofDOFs; j++){
            if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
                distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
        }
        int numofsamples = (int)(distance/(PI/20));
        if(numofsamples < 2){
            printf("the arm is already at the goal\n");
            return;
        }
        *plan = (double**) malloc(numofsamples*sizeof(double*));
        int firstinvalidconf = 1;
        for (i = 0; i < numofsamples; i++){
            (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
            std::vector<double> angles;
            for(j = 0; j < numofDOFs; j++){
                (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
                angles.push_back((*plan)[i][j]);
            }
            
            // if(!planner.IsValidArmConfiguration(angles) && firstinvalidconf)
            // {
            //     firstinvalidconf = 1;
            //     printf("ERROR: Invalid arm configuration!!!\n");
            // }
        }    
        *planlength = numofsamples;
    }
    
    return;
}

//prhs contains input parameters (3): 
//1st is matrix with all the obstacles
//2nd is a row vector of start angles for the arm 
//3nd is a row vector of goal angles for the arm 
//plhs should contain output parameters (2): 
//1st is a 2D matrix plan when each plan[i][j] is the value of jth angle at the ith step of the plan
//(there are D DoF of the arm (that is, D angles). So, j can take values from 0 to D-1
//2nd is planlength (int)
void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
    
    /* Check for proper number of arguments */    
    if (nrhs != 4) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Four input arguments required."); 
    } else if (nlhs != 2) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = (int) mxGetM(MAP_IN);
    int y_size = (int) mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the start and goal angles*/     
    int numofDOFs = (int) (MAX(mxGetM(ARMSTART_IN), mxGetN(ARMSTART_IN)));
    if(numofDOFs <= 1){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "it should be at least 2");         
    }
    double* armstart_anglesV_rad = mxGetPr(ARMSTART_IN);
    if (numofDOFs != MAX(mxGetM(ARMGOAL_IN), mxGetN(ARMGOAL_IN))){
        	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "numofDOFs in startangles is different from goalangles");         
    }
    double* armgoal_anglesV_rad = mxGetPr(ARMGOAL_IN);
 
    //get the planner id
    int planner_id = (int)*mxGetPr(PLANNER_ID_IN);
    if(planner_id < 0 || planner_id > 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidplanner_id",
                "planner id should be between 0 and 3 inclusive");         
    }
    
    //call the planner
    double** plan = NULL;
    int planlength = 0;
    
    // 0: RRT
    // 1: RRT-Connect
    // 2: RRT*
    // 3: PRM
    
    //you can may be call the corresponding planner function here
    //if (planner_id == RRT)
    //{
    //    plannerRRT(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    //}
    
    //dummy planner which only computes interpolated path
    planner(planner_id, map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength); 
    
    printf("planner returned plan of length=%d\n", planlength); 
    
    /* Create return values */
    if(planlength > 0)
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
        int i,j;
        for(i = 0; i < planlength; i++)
        {
            for (j = 0; j < numofDOFs; j++)
            {
                plan_out[j*planlength + i] = plan[i][j];
            }
        }
    }
    else
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);
        //copy the values
        int j;
        for(j = 0; j < numofDOFs; j++)
        {
                plan_out[j] = armstart_anglesV_rad[j];
        }     
    }
    PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL); 
    int* planlength_out = (int*) mxGetPr(PLANLENGTH_OUT);
    *planlength_out = planlength;

    
    return;
    
}
