% INITIALIZE FW SIMULATION

% Thomas Stastny
% 2018.12.12

clear; clc;

% INITIAL CONDITIONS
ic_ned = [0;0;-100];
ic_uvw = [14;0;0];
ic_pqr = zeros(3,1);
ic_rpy = deg2rad([0;5;0]);

% CONTROL INPUTS
ic_u(1) = 0;	% throttle [0,1]
ic_u(2) = 0;	% elevator [-1,1]
ic_u(3) = 0;	% aileron [-1,1]
ic_u(4) = 0;	% rudder [-1,1]

% WIND
wind = [0;0;0];

% GROUND SPEED
ic_vG = angle2dcm(ic_rpy(1),ic_rpy(2),ic_rpy(3),'ZYX')*ic_uvw + wind;
