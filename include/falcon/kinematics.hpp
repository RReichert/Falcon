#include <boost/array.hpp>
#include <boost/math/constants/constants.hpp>

class Kinematics {

  private:

    // mathematical constants
    const double pi = 3.1415926535897932384;

    // motor/model properties 
    const double theta_offset = 35*(pi/180.0);

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

    // NOTE: these readings are based on the "Characterisation of the Novint Falcon Haptic Device for Application as a Robot Manipulator" paper
    //       although pretty simular values are given by libnifalcon

  public:

    // encoding/decoding device data 
    void decodeTheta(const boost::array<int, 3> (&encodedTheta), boost::array<double, 3> (&theta));
    void encodeTorque(const boost::array<double, 3> (&oemga), const boost::array<double, 3> (&torque), boost::array<int, 3> (&encodedTorque));

    // calculation 
    void d_dt(const boost::array<double, 3> (&currentValue), const boost::array<double, 3> (&prevValue), double dt, boost::array<double, 3> (&dValue_dt));

};
