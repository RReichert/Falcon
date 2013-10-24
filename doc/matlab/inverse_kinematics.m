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
  
  % maximum and minimum base angle values
  theta_1_min = decode_theta([min_encoded_theta; min_encoded_theta; min_encoded_theta]);
  theta_1_max = decode_theta([max_encoded_theta; max_encoded_theta; max_encoded_theta]);

  % for each of the arms
  phi=[phi1, phi2, phi3];
  for i=1:3

    % substitution #1
    Rx = rx*cos(phi(i)) + ry*sin(phi(i));
    Ry = -rx*sin(phi(i)) + ry*cos(phi(i));
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
    A = -(X+Z);
    B = 2*Y;
    C = X-Z;

    % calculate the possible configuration for theta 1
    config1 = 2*atan( (-B + sqrt(B^2-4*A*C))  / (2*A) );
    config2 = 2*atan( (-B - sqrt(B^2-4*A*C))  / (2*A) );
    
    % choose the one that is within the possible range
    if(config1 >= theta_1_min(i) && config1 <= theta_1_max(i))
      theta_1 = config1;
    else
      theta_1 = config2;
    end
    
    % calculate theta 2
    %theta_2 = asin( -(Ry + c - r - a*cos(theta_1)) / (L) );
    
    % save result
    theta(i) = theta_1;
    
  end

end