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

  // declare function variables
  double motorOmega, motorTorque;

  // calculate the encoded torque
  for(int x=0; x<3; x++) {
    motorOmega = gain*omega[x];
    motorTorque = -torque[x]/gain;
    encodedTorque[x] = motorTorque / (Ks + Kd*motorOmega);
// XXX: need to check if the polarity of this encoded torque is correct
  }

  // NOTE: derivation was based on the constraint:
  //
  //         arm_torque = arm_radius*F && motor_torque = motor_shaft_radius*F
  //
  //       because F is the common term (interaction force), we can solve for motor torque given theta
  //       following that we use the constraint expressed in decodedTheta and convert omega to motor omega
  //       and use these two values to convert to the motor encoded value

  // NOTE:  the equation to encode motor torque was taken directly from "Characterisation of the Novint Falcon Haptic Device for Application as a Robot Manipulator", however in the future I would like to try and modify it a bit to account for the change in current within the dc motors internal electric component caused by the back emf
}

void Kinematics::d_dt(const boost::array<double, 3> (&currentValue), const boost::array<double, 3> (&prevValue), double dt, boost::array<double, 3> (&dValue_dt)) {
  for(int x=0; x<3; x++) {
    dValue_dt[x] = (currentValue[x] - prevValue[x]) / (dt);
  }
}
