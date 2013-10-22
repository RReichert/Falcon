function [encodedTorque] = encode_torque(omega, torque)

% encode torque
%
%  this function will calculate the necissary encoded value for the motor 
%  that will produce the specified torque on the base given the base's
%  angular vecolcity.
%
%  input: torque [omega, torque]
%  output: encoded motor torque

% calculate the equivalant motor angular velocity
motorOmega = gain*omega;

% calculate the equivalant motor torque
motorTorque = -torque/gain;

% encoded motor torque
encodedTorque = motorTorque ./ (Ks + Kd*motorOmega);

end