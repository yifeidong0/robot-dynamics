% Initialize a struct containing the body lengths in meters.
params = struct;
params.l0 = 0.3 + 0.02*rand;
params.l1 = 0.2 + 0.02*rand;
params.l2 = 0.3 + 0.02*rand;
params.l3 = 0.04 + 0.002*rand;
params.l4 = 0.28 + 0.02*rand;
params.l5 = 0.08 + 0.002*rand;
params.L  = params.l2 + params.l4 + params.l5;

% Sampling time used for discrete integration steps, and for visualization 
params.t_s = 0.05;

% Duration of trajectory tracking task
params.T = 100; 

% Generate BICORN CURVE positions
t = 0:params.t_s:params.T;
N = length(t);
a = 0.2;
y_ref   = a * sin(t) + 0.4;
z_ref   = a * cos(t).^2 ./ (2-cos(t)) + 0.2;
v_y_ref = a * cos(t);
v_z_ref = a * (cos(t).^2 .* sin(t) - 4 * cos(t) .* sin(t)) ./ (2-cos(t)).^2;

% Desired angle of foot w.r.t the ground
alpha_ref = 20*pi/180; 


% Initialize a random vector of joint positions.
q = rand(3,1);