function par = robot_set_parameters

%initialize the VRML world parameters of the camera in the robot's head

% the position of the camera in the virtual world
par.cam_position = [1, 2, 2];
% The second int is the robots' shoulder height
% The third int is the negative value of shoulder's projection on the floor depth_position (x-axis).

% the orientation of the camera in the virtual world
par.cam_orientation = [1 0 0 -0.6];
% The last int (the body angle), comes from the simulation

%% Initialize head variables

% the focal length of the optics
par.f = 0.0006;
% the pixel size of the pixelmatrix of the imager
par.pixel_size = 0.00000317;
% The size of the camera image obtained from the vrmlworld
par.imagesize = [576 380];
% the FieldOfView of the virtual camera (see also the vrml file)
par.fov = 1;

% The bounds in polar uv-space to segment an orange ball
par.bounds.theta = [0.5236 1.2566];
par.bounds.r = [20 100];

% The minimum and maximum ball radius in pixels to be searched
% by the Hough transform algorithm
par.ball_radii = [5 20];

% The true ball_radius in meters
par.ball_radius = 0.2;

% The maximum distance to the ball while still being able to pick up the ball

par.maxDistancetoball = 1.2; 

%% initialize body variables

par.gamma = 0;
par.g = 9.81;
par.m1 = 2;
par.m2 = 2;
par.m3 = 5; 
par.m4 = 2;
par.a1 = 1;
par.a2 = 1;
par.a3 = 1;
par.a4 = 0.7;
par.c1 = 0.2;
par.c2 = 0.2;
par.c3 = 0.2;
par.c4 = 0.2;
par.I1 = (1/15)*par.m1*par.a1^2;
par.I2 = (1/15)*par.m2*par.a2^2;
par.I3 = (1/15)*par.m3*par.a3^2;
par.I4 = (1/15)*par.m4*par.a4^2;
