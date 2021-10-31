function [ T_IF ] = jointToEndeffectorPose_solution( q, params )
    % q: a 3x1 vector of generalized coordinates
    % params: a struct of parameters

    % Link lengths (meters)
    l0 = params.l0;
    l1 = params.l1;
    l2 = params.l2;
    l3 = params.l3;
    l4 = params.l4;
    l5 = params.l5;
    L  = params.L;

    % Joint positions
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);

    T_I0 = [0  0  1  0;
            0  1  0  0;
            -1 0  0  L;
            0  0  0  1];

    T_01 = [cos(q1) -sin(q1) 0 0;
            sin(q1)  cos(q1) 0 l0;
            0        0       1 l1;
            0        0       0 1];

    T_12 = [cos(q2) -sin(q2) 0 l2;
            sin(q2)  cos(q2) 0 0;
            0        0       1 0;
            0        0       0 1];

    T_23 = [cos(q3) -sin(q3) 0 l4;
            sin(q3)  cos(q3) 0 l3;
            0        0       1 0;
            0        0       0 1];

    T_3F = [1 0 0 l5;
            0 1 0 0;
            0 0 1 0;
            0 0 0 1];

    T_IF = T_I0*T_01*T_12*T_23*T_3F;
    
end