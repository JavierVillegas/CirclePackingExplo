//
//  bubble.h
//  CirclePackingExplo
//
//  Created by Javier Villegas on 9/18/13.
//
//

#ifndef __CirclePackingExplo__bubble__
#define __CirclePackingExplo__bubble__
#include "ofMain.h"
#include <iostream>

class bubble {

public:
    float x; // x coordinate
    float y; // y coordinate
    float rad; // radious
    float dc; // distance to center
    float velx;
    float vely;
    bubble();
    bubble(float,float, float,float);
    bubble(float,float, float,float, float, ofVec2f);

    friend bool operator>(const bubble& B1, const bubble& B2);
    friend bool operator<(const bubble& B1, const bubble& B2);
};










#endif /* defined(__CirclePackingExplo__bubble__) */
