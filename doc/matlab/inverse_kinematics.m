function [theta] = inverse_kinematics(position)

% inverse kinematics:
%
%  given the end effector inertial position, it finds the base angles.
%
%  input: position in inertial cordinate system
%  return: base angles [arm1, arm2, arm3]

% parameter check
assert(all(size(position) == [3,1]), 'position must be of 3x1 size');

% load variables
variables;

% obtain individual position elements
rx = position(1);
ry = position(2);
rz = position(3);

% for each of the arms
for phi=[phi1, phi2, phi3],

  % substitution #1
  Rx = rx*cos(phi) + ry*sin(phi);
  Ry = -rx*sin(phi) + ry*cos(phi);
  Rz = rz;

  % calculate theta 3
  theta_3 = asin((Rx+f-s) / b);

  % substitution #2
  L = d + b*cos(theta_3) + e;

  % substitution #3
  X = Ry + c - r;
  Y = Rz;
  Z = (a^2 +r^2 + c^2 + Ry^2 + Rz^2 + 2*Ry*c -2*Ry*r - 2*r*c - L^2) / (2*a);

  % substitution
  A = -X;
  B = 2*Y;
  C = -Z-X;

  % calculate theta 1
  gamma = (-B + sqrt(B^2-A*C))  / (2*A);
  theta_1 = 2*atan(gamma);

  % calculate theta 2
  theta_2 = asin( -(Ry + c - r - a*cos(theta_1)) / (L) );

  display theta_1:
  rad2deg(theta_1)

  display solution:
  C_iI = [cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0 ; 0 0 1];
  C_iI*[b*sin(theta_3)-f+s; a*cos(theta_1)-sin(theta_2)*L-c+r; a*sin(theta_1)+cos(theta_2)*L]

end

theta = [0;0;0];

end