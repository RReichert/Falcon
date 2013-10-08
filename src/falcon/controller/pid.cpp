#include "falcon/controller/pid.hpp"

PID::PID(boost::array<double, 3> startPosition) : Controller(startPosition) {

}

boost::array<double, 3> PID::getForce(boost::array<double, 3> currentPosition, boost::array<double, 3> desiredPosition) {
  return boost::array<double, 3>();
}
