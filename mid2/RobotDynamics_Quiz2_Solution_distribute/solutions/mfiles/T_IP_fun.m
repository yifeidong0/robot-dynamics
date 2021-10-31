function T_IP = T_IP_fun(params)

    % Link lengths (meters)
    l0 = params.l0;
    l1 = params.l1;
    l2 = params.l2;
    l3 = params.l3;
    l4 = params.l4;
    l5 = params.l5;
    L  = params.L;

% 
%     T_IP = [1  0  0  l0;
%             0  cos(pi/4) -sin(pi/4)  0.4;
%             0  sin(pi/4)  cos(pi/4)  0.3;
%             0  0  0  1];
        
%     T_IP = [1  0  0  l0;
%             0  1  0  0.4;
%             0  0  1  0.3;
%             0  0  0  1];
                
    T_IP = [1  0  0  l0;
            0  1  0  0.3;
            0  0  1  0.0;
            0  0  0  1];
        


end