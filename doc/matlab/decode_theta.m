function [result] = decode_theta(encodedTheta)

% decode theta:
%
%  given devices encoded angular position value, it coverts the value to
%  its corresponding base angle value.
%  
%  input: encoded value [motor1, motor2, motor3]
%  return: base angle value [arm1, arm2, arm3]

% parameter check
assert(all(size(encodedTheta) == [3,1]), 'encodedTheta must be of 3x1 size');

% load falcon variables
variables;

% calculate corresponding base theta values
result = (2*pi) * (encodedTheta./(slots*states)) / gain + theta_offset;

end