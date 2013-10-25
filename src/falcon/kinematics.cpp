#include "falcon/kinematics.hpp"

void Kinematics::decodeTheta(const boost::array<int, 3> (&encodedTheta), boost::array<double, 3> (&theta)) {
  for(int x=0; x<3; x++) {
    theta[x] = decodeTheta(encodedTheta[x]);
  } 
}

void Kinematics::encodeTorque(const boost::array<double, 3> (&omega), const boost::array<double, 3> (&torque), boost::array<int, 3> (&encodedTorque)) {
  for(int x=0; x<3; x++) {
    encodedTorque[x] = encodeTorque(omega[x], torque[x]);
  }
}

void Kinematics::d_dt(const boost::array<double, 3> (&currentValue), const boost::array<double, 3> (&prevValue), double dt, boost::array<double, 3> (&dValue_dt)) {
  for(int x=0; x<3; x++) {
    dValue_dt[x] = (currentValue[x] - prevValue[x]) / (dt);
  }
}

void Kinematics::inverse_kinematics(const boost::array<double, 3> (&position), boost::array<double, 3> (&theta)) {

  // function variables
  double theta1, theta3;
  double rx, ry, rz;
  double Rx, Ry, Rz;
  double X, Y, Z;
  double A, B, C;
  double L;
  double phi[3] = {phi1, phi2, phi3};
  double config1, config2;

  // obtain each position element
  rx = position[0];
  ry = position[1];
  rz = position[2];

  // loop through each arm and calculate the theta values
  for(int i=0; i<3; i++) {

    Rx = rx*cos(phi[i]) + ry*sin(phi[i]);
    Ry = -rx*sin(phi[i]) + ry*cos(phi[i]);
    Rz = rz;

    theta3 = asin((Rx+f-s) / b);
    L = d + b*cos(theta3) + e;

    X = Ry + c - r;
    Y = Rz;
    Z = (a*a +r*r + c*c + Ry*Ry + Rz*Rz + 2*Ry*c -2*Ry*r - 2*r*c - L*L) / (2*a);

    A = -(X+Z);
    B = 2*Y;
    C = X-Z;

    config1 = 2*atan( (-B + sqrt(B*B-4*A*C))  / (2*A) );
    config2 = 2*atan( (-B - sqrt(B*B-4*A*C))  / (2*A) );
    
    if(config1 >= min_theta && config1 <= max_theta) {
      theta1 = config1;
    } else {
      theta1 = config2;
    } 
    
    theta[i] = theta1;
  }

}
