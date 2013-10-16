#include <boost/array.hpp>
#include <boost/math/constants/constants.hpp>

class Kinematics {

  private:

    // mathematical constants
    const double pi = 3.1415926535897932384;

    // motor/model properties 
    const double theta_offset = 35*(pi/180.0);

    // motor properties
    const double Im = 0;
    const double Ks = 0;
    const double Kd = 0;

    const double gain = 7.62;
    const double slots = 320;
    const double states = 4;

// NEED TO UPDATE XXX

    // mechanical properties
    const double a = 0.060;
    const double b = 0.1025;
    const double c = 0.0157;
    const double d = 0.0115;
    const double e = d;
    const double f = 0.0262;
    const double g = 0.0279;
    const double r = 0.0366;
    const double s = 0.0272;

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
