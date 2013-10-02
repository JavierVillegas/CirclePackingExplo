//
//  bubble.cpp
//  CirclePackingExplo
//
//  Created by Javier Villegas on 9/18/13.
//
//

#include "bubble.h"


bubble::bubble(){
    
    x =0.0;
    y = 0.0;
    rad = 10;
    dc = 100;
    velx =0;
    vely =0;
}
bubble::bubble(float ex, float ey, float erad, float edc){
    
    x = ex;
    y = ey;
    rad = erad;
    dc = edc;
    velx =0;
    vely =0;
}

bubble::bubble(float ex, float ey, float erad, float evelx, float evely, ofVec2f eCenter){
    
    x = ex;
    y = ey;
    rad = erad;
    dc = sqrt((x - eCenter.x)*(x - eCenter.x) +
              (y - eCenter.y)*(y - eCenter.y));
    velx = evelx;
    vely = evely;
}

bool operator> (const bubble &B1, const bubble &B2)
{
    return (B1.dc) > (B2.dc);
}
bool operator< (const bubble &B1, const bubble &B2)
{
    return (B1.dc) < (B2.dc);
}