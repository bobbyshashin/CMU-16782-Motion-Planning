#include "Planner.h"
#include "BresenhamParam.h"
// #include "mex.h"

// Helper functions
void ContXY2Cell(double x, double y, int& px, int& py, int x_size, int y_size) {   
    double cellsize = 1.0;
	//take the nearest cell
	px = (int)(x / (double)(cellsize));
	if(x < 0)
        px = 0;
	if(px >= x_size) 
        px = x_size - 1;

	py = (int)(y / (double)(cellsize));
	if(y < 0) 
        py = 0;
	if(py >= y_size)
        py = y_size - 1;

}

void GetCurrentPoint(const BresenhamParam& params, int& x, int& y) {

    if (params.UsingYIndex) {
        y = params.XIndex;
        x = params.YIndex;
        if (params.Flipped)
            x = -x;
    } else {
        x = params.XIndex;
        y = params.YIndex;
        if (params.Flipped)
            y = -y;
    }

}

int getNextPoint(BresenhamParam& params) {
    if (params.XIndex == params.X2) {
        return 0;
    }

    params.XIndex += params.Increment;

    if (params.DTerm < 0 || (params.Increment < 0 && params.DTerm <= 0)) {
        params.DTerm += params.IncrE;
    } else {
        params.DTerm += params.IncrNE;
        params.YIndex += params.Increment;
    }

  return 1;
}

bool Planner::IsValidLineSegment(double x0, double y0, double x1, double y1) const {

	int nX, nY; 
    int nX0, nY0, nX1, nY1;

	//make sure the line segment is inside the environment
    if(x0 < 0 || x0 >= map_size_x ||
	   x1 < 0 || x1 >= map_size_x ||
	   y0 < 0 || y0 >= map_size_y ||
	   y1 < 0 || y1 >= map_size_y)
	      return false;

    ContXY2Cell(x0, y0, nX0, nY0, map_size_x, map_size_y);
	ContXY2Cell(x1, y1, nX1, nY1, map_size_x, map_size_y);

	//iterate through the points on the segment
	BresenhamParam params = getBresenhamParam(nX0, nY0, nX1, nY1);
	do {
		GetCurrentPoint(params, nX, nY);
		if(map[nX][nY] == 1)
            return false;
	} while (getNextPoint(params));

	return true;
}


bool Planner::IsValidArmConfiguration(const std::vector<double>& angles) const {

    // check if dof matches
    if (angles.size() != DOF) {
        // mexPrintf("Number of DOF does not match in IsValidArmConfiguration!");
        return false;
    }

    double x0,y0,x1,y1;

 	//iterate through all the links starting with the base
	x1 = ((double)map_size_x) / 2.0;
    y1 = 0;
	for (int i=0; i<DOF; ++i) {
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + link_length_cells * cos(2 * pi - angles[i]);
		y1 = y0 - link_length_cells * sin(2 * pi - angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1))
			return false;
	}   

    return true;
}

bool Planner::isValidEdge(const std::vector<double>& config1, const std::vector<double>& config2, int interpolation_num) {
    std::vector<double> direction;
    for (int i=0; i<DOF; ++i) {
        direction.push_back(config2[i] - config1[i]);
    }

    std::vector<double> inter(config1);
    int n = 1;
    // int interpolation_num = 100;
    while (n <= interpolation_num) {
        for (int i=0; i<DOF; ++i) {
            inter[i] = config1[i] + ((double)n / (double)interpolation_num) * direction[i];
        }
        if (!IsValidArmConfiguration(inter)) {
           return false;
        }
        ++n;
    }
    return true;

}


bool Planner::chooseGoal() {
    double sample = (double)rand() / (RAND_MAX);
    return (sample <= goal_bias_weight);

}

std::vector<double> Planner::generateRandomSample() {
    std::vector<double> config;
    for (int i=0; i<DOF; ++i) {
        // generate a random configuration from 0 to 2pi
        double rand_config = (double)rand() / (RAND_MAX) * 2 * pi;
        config.push_back(rand_config);
    }
    return config;
}

double Planner::euclideanDist(std::vector<double> config1, std::vector<double> config2) {
    double distance = 0;
    for (int i=0; i<DOF; ++i) {
        distance += (config1[i] - config2[i]) * (config1[i] - config2[i]);
    }
    return sqrt(distance);
}
