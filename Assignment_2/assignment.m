function assignment

    par = robot_set_parameters;
    par.run_type = 'learn';
    par = swingup(par);
    
    par.run_type = 'test';
    [par, ta, xa] = swingup(par);
       
    robot_animate_provided(ta, xa, par)
end
