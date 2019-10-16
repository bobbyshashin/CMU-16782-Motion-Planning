#include <math.h>

struct BresenhamParam {
        int X1, Y1;
        int X2, Y2;
        int Increment;
        int UsingYIndex;
        int DeltaX, DeltaY;
        int DTerm;
        int IncrE, IncrNE;
        int XIndex, YIndex;
        int Flipped;

};

BresenhamParam getBresenhamParam(int p1x, int p1y, int p2x, int p2y) {
    BresenhamParam params;

    params.UsingYIndex = 0;

    if (fabs((double)(p2y - p1y) / (double)(p2x - p1x)) > 1)
        (params.UsingYIndex)++;

    if (params.UsingYIndex) {
        params.Y1 = p1x;
        params.X1 = p1y;
        params.Y2 = p2x;
        params.X2 = p2y;
    } else {
        params.X1 = p1x;
        params.Y1 = p1y;
        params.X2 = p2x;
        params.Y2 = p2y;
    }

    if ((p2x - p1x) * (p2y - p1y) < 0) {
        params.Flipped = 1;
        params.Y1 = -params.Y1;
        params.Y2 = -params.Y2;
    } else {
        params.Flipped = 0;
    }
        

    if (params.X2 > params.X1)
        params.Increment = 1;
    else
        params.Increment = -1;

    params.DeltaX = params.X2 - params.X1;
    params.DeltaY = params.Y2 - params.Y1;

    params.IncrE = 2 * params.DeltaY * params.Increment;
    params.IncrNE =2 * (params.DeltaY - params.DeltaX) * params.Increment;
    params.DTerm = (2 * params.DeltaY - params.DeltaX) * params.Increment;

    params.XIndex = params.X1;
    params.YIndex = params.Y1;

    return params;

}