#include <boost/array.hpp>
#include <boost/math/constants/constants.hpp>

class Kinematics {

  private:

    // mathematical constants
    const double pi = 3.1415926535897932384;

    // coordinate systems
    const double phi1 = 15*(pi/180.0);
    const double phi2 = 255*(pi/180.0);
    const double phi3 = 135*(pi/180.0);

    // motor/model properties 
    const double theta_offset = 40*(pi/180.0);
    const double min_theta = decodeTheta(-1600);
    const double max_theta = decodeTheta(1600);

    // motor properties
    const double Ks = 1.9835e-5;
    const double Kd = 1.3779e-8;

    const double gain = 7.62;
    const double slots = 320;
    const double states = 4;

    // mechanical properties
    const double a = 60e-3;
    const double b = 102.5e-3;
    const double c = 15.7e-3;
    const double d = 11.5e-3;
    const double e = d;
    const double f = 26.2e-3;
    const double g = 27.9e-3;
    const double r = 36.6e-3;
    const double s = 27.2e-3;

    // NOTE: all these readings are in SI unites
    // NOTE: these readings are based on the "Characterisation of the Novint Falcon Haptic Device 
    //       for Application as a Robot Manipulator" paper although pretty simular values are given
    //       by libnifalcon. contrary the what the paper says, I have noticed that the offset angle
    //       is actually 40 deg rather then the proposed 35 and 50 from the two sources.

  public:

    // decoding device theta value
    void decodeTheta(const boost::array<int, 3> (&encodedTheta), boost::array<double, 3> (&theta));
    inline double decodeTheta(const int encodedTheta) {
      return (2*pi) * (encodedTheta/(slots*states)) / gain + theta_offset;
    }

    // encode device torque value
    void encodeTorque(const boost::array<double, 3> (&omega), const boost::array<double, 3> (&torque), boost::array<int, 3> (&encodedTorque));
    inline int encodeTorque(const double omega, const double torque) {
      double motorOmega = gain*omega;
      double motorTorque = -torque/gain;
      return motorTorque / (Ks + Kd*motorOmega);
    }

    // NOTE:  the equation to encode motor torque was taken directly from "Characterisation of the Novint Falcon Haptic Device for Application as a Robot Manipulator", however in the future I would like to try and modify it a bit to account for the change in current within the dc motors internal electric component caused by the back emf

    // calculate differntial of value
    void d_dt(const boost::array<double, 3> (&currentValue), const boost::array<double, 3> (&prevValue), double dt, boost::array<double, 3> (&dValue_dt));

    // inverse kinematics
    void inverse_kinematics(const boost::array<double, 3> (&position), boost::array<double, 3> (&theta));
};
