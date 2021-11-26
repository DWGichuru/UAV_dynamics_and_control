function out = R_b_v(phi, theta, psi)
R_b_v2 = [
    1   0         0;
    0   cos(phi)  sin(phi);
    0   -sin(phi) cos(phi)
    ];
R_v2_v1 = [
    cos(theta)  0   -sin(theta);
    0           1   0;
    sin(theta)  0   cos(theta);
    ];
R_v1_v = [
    cos(psi)    sin(psi)    0;
    -sin(psi)   cos(psi)    0;
    0           0           1;
    ];
out = R_b_v2*R_v2_v1*R_v1_v;
end

