function [position] = test_forward_kinematics(theta_11, theta_12, theta_13)

% load variables
variables;

% calculate transformation matrix for coordinate system 1
C_1I = [cos(phi1) -sin(phi1) 0; sin(phi1) cos(phi1) 0 ; 0 0 1];

% calculate the relative position to the end effector
rx = s + b*sin(theta_13) - f;
ry = r + a*cos(theta_11) -sin(theta_12)*(d+b*cos(theta_13)+e) - c;
rz =     a*sin(theta_11) + cos(theta_12)*(d+b*cos(theta_13)+e);
position = [rx; ry; rz];

% display results
%position = C_1I*[rx;ry;rz];

end