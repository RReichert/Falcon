function forward_kinematics_newton_raphson()

% forward kinematics:
%
%  this function will present the C code needed to implement the forward
%  kinematic algorithm.

% load symbolic variables
sym_variables;

% create angle/position variables
syms rx ry rz real;
syms theta_11 theta_12 theta_13 real;
syms theta_21 theta_22 theta_23 real;
syms theta_31 theta_32 theta_33 real;

% define transformation matrix for each relative coordinate system
C_1I = [cos(phi1) -sin(phi1) 0; sin(phi1) cos(phi1) 0 ; 0 0 1];
C_2I = [cos(phi2) -sin(phi2) 0; sin(phi2) cos(phi2) 0 ; 0 0 1];
C_3I = [cos(phi3) -sin(phi3) 0; sin(phi3) cos(phi3) 0 ; 0 0 1];

% calculate the relative position to the end effector
r1 = [ s + b*sin(theta_13) - f ; r + a*cos(theta_11) - sin(theta_12)*(d+b*cos(theta_13)+e) - c ; a*sin(theta_11) + cos(theta_12)*(d+b*cos(theta_13)+e) ];
r2 = [ s + b*sin(theta_23) - f ; r + a*cos(theta_21) - sin(theta_22)*(d+b*cos(theta_23)+e) - c ; a*sin(theta_21) + cos(theta_22)*(d+b*cos(theta_23)+e) ];
r3 = [ s + b*sin(theta_33) - f ; r + a*cos(theta_31) - sin(theta_32)*(d+b*cos(theta_33)+e) - c ; a*sin(theta_31) + cos(theta_32)*(d+b*cos(theta_33)+e) ];
    
% create kinematic constraint equations
Eq1 = C_2I*r2 - C_1I*r1;
Eq2 = C_3I*r3 - C_1I*r1;
Eq3 = C_1I*r1 - [rx;ry;rz];

% setup necisary variables
F = [Eq1(1) ; Eq1(2) ; Eq1(3) ; Eq2(1) ; Eq2(2) ; Eq2(3) ; Eq3(1) ; Eq3(2) ; Eq3(3)];
F_dash = [diff(F,rx), diff(F,ry), diff(F,rz), diff(F,theta_12), diff(F,theta_13), diff(F,theta_22), diff(F,theta_23), diff(F,theta_32), diff(F,theta_33)];

% simplify results
F = simplify(F);
F_dash = simplify(F_dash);

% display guess
display 'Guess: rx, ry, rz, theta_12, theta_13, theta_22, theta_23, theta_32, theta_33'

% convert to C code
disp(ccode(F));
disp(ccode(F_dash));

end
