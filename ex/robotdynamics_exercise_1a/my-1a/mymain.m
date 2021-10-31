q = zeros(6,1);
q = pi/6 + q;

I_r_IE = jointToPosition(q);
C_IE = jointToRotMat(q);
quat = rotMatToQuat(q);
R = quatToRotMat(quat);
quat1 = jointToQuat(q);