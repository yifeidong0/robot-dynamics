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
  I_J = jointToGeometricJacobian(q, params);
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
  q_0 = [0.5968; 1.0816; 0.8688];
  q_ik = inverse_kinematics(0.6814, 0.6599, 2.1297, q_0, 1e-6, params);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end