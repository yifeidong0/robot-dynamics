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

  t1=[cos(tY)*cos(tZ),-cos(tY)*sin(tZ),sin(tY),0];
  t2=[ cos(tX)*sin(tZ)+cos(tZ)*sin(tX)*sin(tY), cos(tX)*cos(tZ)-sin(tZ)*sin(tX)*sin(tY),-cos(tY)*sin(tX),0];
  t3=[sin(tX)*sin(tZ)-cos(tZ)*cos(tX)*sin(tY),sin(tX)*cos(tZ)+sin(tZ)*cos(tX)*sin(tY),cos(tY)*cos(tX),0];
  t4=[0, 0, 0, 1];
  T_0B = [t1;t2;t3;t4];
  T_IF = T_I0 * T_0B * T_B1 * T_12 * T_23 * T_3F;
  C_IF = T_IF(1:3,1:3);
    
end