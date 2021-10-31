function [ C_IF ] = footFrameOrientationWithBaseRotation(thetaX, thetaY, thetaZ, q, params) 
  % C_IF is the rotation matrix describing the foot frame orientation
  % with respect to the inertial frame when the base orientation is not fixed. 
  
  % The base's new orientation is parametrized using the XYZ Euler-Angles: 
  % thetaX     : angle of rotation around x-axis of frame {0}
  % thetaY     : angle of rotation around y_axis of frame {0'}
  % thetaZ     : angle of rotation around z-axis of frame {0''}
  
  tX = thetaX;
  tY = thetaY;
  tZ = thetaZ;
  
  % Get the homogeneous transforms between each pair of coordinate frames.
  T_I0 = T_I0_solution(q, params);
  T_B1 = T_B1_solution(q, params);
  T_12 = T_12_solution(q, params);
  T_23 = T_23_solution(q, params);
  T_3F = T_3F_solution(q, params);
  
  % Implement your solution here.
  C_IF = [];
    
end