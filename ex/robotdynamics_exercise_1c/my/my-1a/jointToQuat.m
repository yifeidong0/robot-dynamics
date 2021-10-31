function quat = jointToQuat(q)
    C = jointToRotMat(q);
    quat = rotMatToQuat(C);
end
