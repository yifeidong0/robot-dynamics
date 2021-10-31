function J_P = jointToPosJac(q)
    T01 = jointToTransform01(q);
    T12 = jointToTransform12(q);
    T23 = jointToTransform23(q);
    T34 = jointToTransform34(q);
    T45 = jointToTransform45(q);
    T56 = jointToTransform56(q);
    
    C_I1 = T01(1:3,1:3);
    C_I2 = C_I1*T12(1:3,1:3);
    C_I3 = C_I2*T23(1:3,1:3);
    C_I4 = C_I3*T34(1:3,1:3);
    C_I5 = C_I4*T45(1:3,1:3);
    C_I6 = C_I5*T56(1:3,1:3);
    
    n1 = [0 0 1]';
    n2 = C_I1*[0 1 0]';
    n3 = C_I2*[0 1 0]';
    n4 = C_I3*[1 0 0]';
    n5 = C_I4*[0 1 0]';
    n6 = C_I5*[1 0 0]';
    
    r12_I = C_I1*[0 0 0.145]';
    r23_I = C_I2*[0 0 0.270]';
    r34_I = C_I3*[0.134 0 0.070]';
    r45_I = C_I4*[0.168 0 0]';
    r56_I = C_I5*[0.072 0 0]';
    r6E_I = C_I6*[0 0 0]';
    
    r1E_I = r12_I+r23_I+r34_I+r45_I+r56_I+r6E_I;
    r2E_I = r23_I+r34_I+r45_I+r56_I+r6E_I;
    r3E_I = r34_I+r45_I+r56_I+r6E_I;
    r4E_I = r45_I+r56_I+r6E_I;
    r5E_I = r56_I+r6E_I;
    
    J_P = [cross(n1,r1E_I),cross(n2,r2E_I),cross(n3,r3E_I),...
        cross(n4,r4E_I),cross(n5,r5E_I),cross(n6,r6E_I)];
    
end