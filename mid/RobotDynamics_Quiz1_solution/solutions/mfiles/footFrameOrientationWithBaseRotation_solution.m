function [ C_IF ] = footFrameOrientationWithBaseRotation_solution(thetaX, thetaY, thetaZ, q, params) 
  % C_IF is the rotation matrix describing the foot frame orientation
  % with respect to the inertial frame when the base orientation is not fixed. 
  
  % The base's new orientation is parametrized using the XYZ Euler-Angles: 
  % thetaX     : angle of rotation around x-axis of frame {0}
  % thetaY     : angle of rotation around new y_axis of frame {0}
  % thetaZ     : angle of rotation around new z-axis of frame {0}
  
  tX = thetaX;
  tY = thetaY;
  tZ = thetaZ;
  
  % Get the homogeneous transforms between each pair of coordinate frames.
  T_I0 = T_I0_solution(q, params);
  T_B1 = T_B1_solution(q, params);
  T_12 = T_12_solution(q, params);
  T_23 = T_23_solution(q, params);
  T_3F = T_3F_solution(q, params);
  
  T_0_thetaX = [1     0        0     0
                0  cos(tX) -sin(tX)  0;
                0  sin(tX)  cos(tX)  0;
                0     0        0     1];
            
  T_0_thetaY = [cos(tY)  0   sin(tY) 0;
                0        1      0    0;
                -sin(tY) 0   cos(tY) 0;
                0        0      0    1];
            
  T_0_thetaZ = [cos(tZ) -sin(tZ) 0   0;
                sin(tZ)  cos(tZ) 0   0;
                0        0       1   0;
                0        0       0   1];
            
  T_IF = T_I0 * T_0_thetaX * T_0_thetaY * T_0_thetaZ * T_B1 * T_12 * T_23 * T_3F;
  
  C_IF = T_IF(1:3,1:3);    
end