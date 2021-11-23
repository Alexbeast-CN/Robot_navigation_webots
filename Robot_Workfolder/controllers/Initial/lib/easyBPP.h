#ifndef _EASYBPP_H
#define _EASYBPP_H

#include "Odometry.h"

class easyBPP
{
private:
    /* data */
public:
    easyBPP(/* args */);
    ~easyBPP();
    void Motion();

};

void easyBPP::Motion()
{
     // Motion logic
    if (mat.Point(ahead_x,ahead_y)==1)
    {
      if (mat.Point(map_x,y_right)==1)
      {
        if (mat.Point(map_x,y_left)==1)
          SweepBot->stop();
        else
          SweepBot->turn_around_left(Regular_speed);
      }
      else if (mat.Point(map_x,y_left)==1)
      {
        if (mat.Point(map_x,y_right)==1)
          SweepBot->stop();
        else
          SweepBot->turn_around_right(Regular_speed);
      }
      else 
        SweepBot->turn_around_right(Regular_speed);
    }
    else
      SweepBot->forward(Regular_speed);
}

#endif