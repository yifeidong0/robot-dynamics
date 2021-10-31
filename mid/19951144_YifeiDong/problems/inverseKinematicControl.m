function [ q_next ] = inverseKinematicControl( q, p_des, w_des, params )  
    % Inputs:
    %  q             : current joint angles (3x1)
    %  p_des         : desired foot pose (3x1)
    %  w_des         : desired foot twist (3x1)
    %  params        : used to access the sampling time

    % Output:
    %  q_next        : new joint angles (3x1)
    
    % ts: Sampling time in seconds      
    t_s = params.t_s;
    
    % Choose a proportional controller gain
    K_p = 5;
    
    % Choose a pseudo_inverse damping coefficient
    lambda = 0.1;
    
    % Get the homogeneous transforms between each pair of coordinate frames.
    T_I0 = T_I0_solution(q, params);
    T_01 = T_01_solution(q, params);
    T_12 = T_12_solution(q, params);
    T_23 = T_23_solution(q, params);
    T_3F = T_3F_solution(q, params);
    
    T_IF = T_I0 * T_01 * T_12 * T_23 * T_3F;
    
    % The current foot pose w.r.t the inertial frame {I} 
    p_current = [T_IF(2:3,4); sum(q)];
    % Implement your solution here...

    JA = jointToEndeffectorAnalyticJacobian_solution(q,params);
    
    w_command = w_des + K_p*(p_des - p_current); %key point: how to decide the current end-effector commanding velocity
    Dq = pseudoInverseMat_solution(JA, lambda) * w_command;
    q_next = q + Dq.*t_s;
    
end