function B_r = rotVecWithQuat(q_BA,A_r)
    p_A_r = [0;A_r];
    t = mycross(q_BA,p_A_r);
    p_B_r = mycross(t,q_BA');
    B_r = p_B_r(2:4,:);
end