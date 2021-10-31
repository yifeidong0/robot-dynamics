% Run this script after implementing all required functions.
% This will allow you to see a visualization of the robot-leg
% tracking a reference trajectory. 

close all;
clear params;
init_workspace; % also initializes params

% Set to 0 to test your own solution, set to 1 to see the final result
use_solution = 1;

%% Control Loop
Q = [];
q_current = [-pi/1.2;pi/2.5;pi/4];

for j = 1:N
    p_des = [y_ref(j);z_ref(j);alpha_ref];
    w_des = [v_y_ref(j);v_z_ref(j);0];
    switch use_solution
        case 0
            q_next = inverseKinematicControl(q_current, p_des, w_des, params);
        case 1
            q_next = inverseKinematicControl_solution(q_current, p_des, w_des, params);
    end
    Q = [Q q_next];
    q_current = q_next;
end

% Visualize Robot (if visualization gets stuck, press Ctrl + C in the Command Window)
positionRef = [y_ref;z_ref];
animate_robot_leg(Q, positionRef, params);

