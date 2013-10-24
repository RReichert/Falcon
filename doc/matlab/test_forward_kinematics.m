function [position] = test_forward_kinematics(phi, theta_1, theta_2, theta_3)

% load variables
variables;

% calculate transformation matrix for coordinate system 1
C_iI = [cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0 ; 0 0 1];

% calculate the relative position to the end effector
rx = s + b*sin(theta_3) - f;
ry = r + a*cos(theta_1) -sin(theta_2)*(d+b*cos(theta_3)+e) - c;
rz =     a*sin(theta_1) + cos(theta_2)*(d+b*cos(theta_3)+e);

% display results
position = C_iI*[rx;ry;rz];

end