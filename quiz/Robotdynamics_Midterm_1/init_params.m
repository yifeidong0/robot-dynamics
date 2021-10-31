% Initialize a struct containing the link lengths in meters.
params = struct;
params.l0 = 0.3;
params.l1 = 0.5;
params.l2 = 0.25;
params.l3 = 0.25;

% Initialize a random vector of joint positions.
q = rand(3,1);