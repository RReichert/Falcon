#include "falcon/controller/controller.hpp"

Controller::Controller(boost::array<double, 3> startPosition) {
  this->position = startPosition;
}
