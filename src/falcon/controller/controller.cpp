#include "falcon/controller/controller.hpp"

Falcon_Controller::Falcon_Controller(boost::array<double, 3> startPosition) {
  this->position = startPosition;
}
