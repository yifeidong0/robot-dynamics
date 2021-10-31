function animate_robot_leg(Q, positionRef, params)

    close all;

    % Link lengths (meters)
    l0 = params.l0;
    l1 = params.l1;
    l2 = params.l2;
    l3 = params.l3;
    l4 = params.l4;
    l5 = params.l5;
    L  = params.L;
    
    % Sampling time for the visualization
    t_s = params.t_s;
    
    
    % Length of time series for joint angles
    N = size(Q,2);
    
    for j = 1:N
      q = Q(:,j);
      
      T_I0 = T_I0_solution(q, params);
      T_01 = T_01_solution(q, params);
      T_12 = T_12_solution(q, params);
      T_2K = T_2K_solution(q, params);
      T_23 = T_23_solution(q, params);
      T_3F = T_3F_solution(q, params);

      T_I1 = T_I0*T_01;
      T_I2 = T_I1*T_12;
      T_IK = T_I2*T_2K;
      T_I3 = T_I2*T_23;
      T_IF = T_I3*T_3F;

      I_r_I1 = T_I1(2:3,4);
      I_r_I2 = T_I2(2:3,4);
      I_r_IK = T_IK(2:3,4);
      I_r_I3 = T_I3(2:3,4);
      I_r_IF = T_IF(2:3,4);
      
      plot(I_r_I1(1), I_r_I1(2), 'ro', ...
           I_r_I2(1), I_r_I2(2), 'ro', ...
           I_r_I3(1), I_r_I3(2), 'ro', ...
           I_r_IF(1), I_r_IF(2), 'k.', 'MarkerSize',5);  
       
      hold on;
      
      plot([-l0-0.07 l0+0.07], [L L], 'k--', ...
           [I_r_I1(1) I_r_I2(1)], [I_r_I1(2) I_r_I2(2)], 'g', ...
           [I_r_I2(1) I_r_IK(1)], [I_r_I2(2) I_r_IK(2)], 'c', ...
           [I_r_IK(1) I_r_I3(1)], [I_r_IK(2) I_r_I3(2)], 'c', ...
           [I_r_I3(1) I_r_IF(1)], [I_r_I3(2) I_r_IF(2)], 'b', 'LineWidth', 1.5);
       
      plot(positionRef(1,:),positionRef(2,:));
       
      hold off;
       
      axis([-l0-L,l0+L,-0.5,0.8]);
      drawnow;
      
      pause(t_s);
    end
    
end