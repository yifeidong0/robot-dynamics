% Initialize the workspace.
init_params;

%% Exercise 1.
disp('Running exercise 1...');
try
  T_IE = jointToEndeffectorPose(q, params);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end



%% Exercise 2.
disp('Running exercise 2...');
try
  I_J = jointToKneeGeometricJacobian(q, params);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end

%% Exercise 3.
disp('Running exercise 3...');
try
  q = [0.5968; 1.0816; 0.8688];
  p_des = [0.0; 0.0; 0.0];
  w_des = [0.0; 0.0; 0.0];
  q_ik = inverseKinematicControl(q, p_des, w_des, params);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end

%% Exercise 4.
disp('Running exercise 4...');
try
  C_F = footFrameOrientationWithBaseRotation(0.0, 0.0, 0.0, q, params);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end