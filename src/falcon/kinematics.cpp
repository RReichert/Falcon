#include "falcon/kinematics.hpp"

void Kinematics::decodeTheta(const boost::array<int, 3> (&encodedTheta), boost::array<double, 3> (&theta)) {

  for(int x=0; x<3; x++) {
    theta[x] = (2*pi) * (encodedTheta[x]/(slots*states)) / gain + theta_offset;
  } 

  // NOTE: derivation was based on the constraint:
  //
  //         arm_radius*theta = motor_shaft_radius*motor_theta
  //
  //       with gain = arm_radius/motor_shaft_radius

  // NOTE: motor_theta is calculated based on the fact that we have states*slots per motor revolution
}

void Kinematics::encodeTorque(const boost::array<double, 3> (&omega), const boost::array<double, 3> (&torque), boost::array<int, 3> (&encodedTorque)) {
  for(int x=0; x<3; x++) {
    encodedTorque[x] = torque[x] / (Ks + Kd*omega[x]);
  }

  // NOTE: this equation was taken directly from "Characterisation of the Novint Falcon Haptic Device for Application as a Robot Manipulator", however a possible modification would be nice to account for the change in current within the dc motors internal electric component. The current mathematical model is good enough to approximate both the static/dynamic nature of the motor. 
}

void Kinematics::d_dt(const boost::array<double, 3> (&currentValue), const boost::array<double, 3> (&prevValue), double dt, boost::array<double, 3> (&dValue_dt)) {
  for(int x=0; x<3; x++) {
    dValue_dt[x] = (currentValue[x] - prevValue[x]) / (dt);
  }
}
