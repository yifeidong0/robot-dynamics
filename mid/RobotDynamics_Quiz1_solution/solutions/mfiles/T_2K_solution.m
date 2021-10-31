function [ T_2K ] = T_2K_solution( q, params )
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

    T_2K = [1  0  0  l3;
            0  1  0  0;
            0  0  1  0;
            0  0  0  1];

end