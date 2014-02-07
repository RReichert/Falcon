#pragma once

#include <cassert>
#include <complex>
#include <algorithm>
#include <boost/array.hpp>
#include <boost/log/trivial.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/math/constants/constants.hpp>

class Kinematics {

  private:

    const double pi = 3.1415926535897932384;

    const double phi1 = 15*(pi/180.0);
    const double phi2 = 255*(pi/180.0);
    const double phi3 = 135*(pi/180.0);

    const double theta_offset = 40*(pi/180.0);
    const double min_theta = decodeTheta(-1600);
    const double max_theta = decodeTheta(1600);

    const double Ks = 1.9835e-5;
    const double Kd = 1.3779e-8;

    const double gain = 7.62;
    const double num_slots = 320;
    const double num_states = 4;

    const double a = 60e-3;
    const double b = 102.5e-3;
    const double c = 15.7e-3;
    const double d = 11.5e-3;
    const double e = d;
    const double f = 26.2e-3;
    const double g = 27.9e-3;
    const double r = 36.6e-3;
    const double s = 27.2e-3;

    const double tolerance = 1e-6;
    const unsigned int max_iterations = 15;

    // NOTE: all these readings are in SI unites
    // NOTE: these readings are based on the "Characterisation of the Novint Falcon Haptic Device
    //       for Application as a Robot Manipulator" paper although pretty simular values are given
    //       by libnifalcon. contrary the what the paper says, I have noticed that the offset angle
    //       is actually 40 deg rather then the proposed 35 and 50 from the two sources.

  public:

    void decodeTheta(const boost::array<int, 3> (&encodedTheta), boost::array<double, 3> (&theta));
    inline double decodeTheta(const int encodedTheta) {
      return (2*pi) * (encodedTheta/(num_slots*num_states)) / gain + theta_offset;
    }

    void encodeTorque(const boost::array<double, 3> (&omega), const boost::array<double, 3> (&torque), boost::array<int, 3> (&encodedTorque));
    inline int encodeTorque(const double omega, const double torque) {
      double motorOmega = gain*omega;
      double motorTorque = -torque/gain;
      return motorTorque / (Ks + Kd*motorOmega);
    }

    // NOTE:  the equation to encode motor torque was taken directly from "Characterisation of the Novint Falcon Haptic Device for Application as a Robot Manipulator", however in the future I would like to try and modify it a bit to account for the change in current within the dc motors internal electric component caused by the back emf

    void d_dt(const boost::array<double, 3> (&currentValue), const boost::array<double, 3> (&prevValue), double dt, boost::array<double, 3> (&dValue_dt));

    bool inverse_kinematics(const boost::array<double, 3> (&position), boost::array<double, 3> (&theta));
    bool inverse_kinematics(const boost::array<double, 3> (&position), boost::array<double, 9> (&thetas));

    // NOTE: the first inverse kinematics only returns the base angles, while the second one returns all the
    //       novint falcon's angles [theta_11, theta_12, .. , theta_32, theta__33]

    bool forward_kinematics(const boost::array<double, 3> (&theta), const boost::array<double, 3> (&positionEstimate), boost::array<double, 3> (&position));

  private:

    double max_error(const boost::numeric::ublas::matrix<double> (&a), const boost::numeric::ublas::matrix<double> (&b));
    bool inverse_matrix(const boost::numeric::ublas::matrix<double> (&m), boost::numeric::ublas::matrix<double> (&inv_m));
    void constraint_matrix(const boost::array<double, 9> (&thetas), const boost::array<double, 3> (&position), boost::numeric::ublas::matrix<double> (&fn));
    void constraint_matrix_dash(const boost::array<double, 9> (&thetas), boost::numeric::ublas::matrix<double> (&fn_dash));
};