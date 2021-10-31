function [ q ] = inverse_kinematics( x_E_des, y_E_des, alpha_des, q_0, tol, params )
  % x_E_des: 1x1 desired x position of end-effector
  % y_E_des: 1x1 desired y position of end-effector
  % alpha_des: 1x1 desired alpha angle of end-effector
  % q_0: 3x1 initial guess for joint angles
  % tol: 1x1 tolerance to use as termination criterion
  %      The tolerance should be used as:
  %      norm([x_E_des; y_E_des; alpha_des] - [x_E; y_E; alpha]) < tol
  % params: a struct of parameters
  
  % Returns a vector of joint angles q (3x1) which
  % achieves the desired task space pose.
  
  it = 0;
  max_it = 100;       % Set the maximum number of iterations. 
lambda = 0.001;     % Damping factor.
alpha = 0.5;        % Update rate

% 1. start configuration
q = q_0;

% 2. Iterate until terminating condition.
  while (it==0 || (norm(dxe)>tol && it < max_it))
    % 3. evaluate Jacobian for current q
    I_J = jointToAnalyticJacobian_solution(q, params);
    
    % 4. Update the psuedo inverse
    I_J_pinv = pseudoInverseMat_solution(I_J, lambda);
    
    % 5. Find the end-effector configuration error vector
    % position error
    I_r_IE = jointToPosition_solution(q);
    I_r_IE_des=[x_E_des, y_E_des,0];
    dr = I_r_IE_des - I_r_IE; 
    % rotation error
    C_IE = jointToRotMat_solution(q);
    C_IE_des = [cos(alpha_des), -sin(alpha_des), 0  ;
         sin(alpha_des),  cos(alpha_des), 0;
              0,       0, 1 ];
    C_err = C_IE_des*C_IE';
    dph = rotMatToRotVec_solution(C_err); 
    % 6D error
    dxe = [dr; dph];
    
    % 6. Update the generalized coordinates
    q = q + alpha*I_J_pinv*dxe;
     
    it = it+1;
  end
end