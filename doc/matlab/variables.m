% coordinate system
phi1 = deg2rad(15);
phi2 = deg2rad(255);
phi3 = deg2rad(135);

% motor/model properties 
theta_offset = deg2rad(40);
max_encoded_theta = 1600;
min_encoded_theta = -1600;

% motor properties
Ks = 1.9835e-5;
Kd = 1.3779e-8;

gain = 7.62;
slots = 320;
states = 4;

% mechanical properties
a = 60e-3;
b = 102.5e-3;
c = 15.7e-3;
d = 11.5e-3;
e = d;
f = 26.2e-3;
g = 27.9e-3;
r = 36.6e-3;
s = 27.2e-3;