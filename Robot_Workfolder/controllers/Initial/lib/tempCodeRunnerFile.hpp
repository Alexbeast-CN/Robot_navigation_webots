      float Current_left = SB->leftposition();
        float Current_right = SB->rightposition();

        l_delta_angle = Current_left - l_last_angle;
        r_delta_angle = Current_right - r_last_angle;
  
        l_last_angle = Current_left;
        r_last_angle = Current_right;

        float forward_contribution = ((WHEEL_RADIUS * l_delta_angle)/2) + ((WHEEL_RADIUS * r_delta_angle)/2);
        float theta_contribution = (l_delta_angle-r_delta_angle)*(0.5*(WHEEL_RADIUS)/ROBOT_RADIUS);
  
        x  = x + (forward_contribution * cos(th));
        y  = y + (forward_contribution * sin(th));
        th = th - theta_contribution;

        if(th > (2*M_PI))
        {
        th = th - (2*M_PI);