function [theta] = inverse_kinematics(position)

% inverse kinematics:
%
%  given the end effector inertial position, it finds the base angles.
%
%  input: position in inertial cordinate system
%  return: base angles [arm1, arm2, arm3]

% load variables
variables;

% transformation angles
phi = [phi1, phi2, phi3];

% substitution
rx = position(1);
ry = position(2);
rz = position(3);

% substitution
Rx = rx*cos(phi) + ry*sin(phi);
Ry = -rx*sin(phi) + ry*cos(phi);
Rz = rz;

% calculate theta_3
theta_3 = arcsin((f+Rx) / b);

% substitution 
L = d + b*cos(theta_3) + e;

% substitution
X = 2*a*Ry + 2*a*c;
Y = 2*a*Rz;
Z = a^2 + c^2 + Ry.^2 + Rz.^2 + 2*Ry*c - L.^2;

% substitution
A = 1;
B = -2*Y./X;
C = Z./X-1;

% calculate base angle
gamma = (-B + sqrt(B.^2-A.*C))  ./ (2*A);
2*atan(gamma)
gamma = (-B + sqrt(B.^2-A.*C))  ./ (2*A);
2*atan(gamma)

theta = [0;0;0];

end