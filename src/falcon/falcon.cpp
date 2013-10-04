#include "falcon/falcon.hpp"

Falcon::Falcon() {
/*
  // check how many devices are connected
  unsigned int deviceCount;
  device.getDeviceCount(deviceCount);

  // report if no falcon was found
  if(deviceCount == 0) {
    error = "no device found";
    return;
  }

  // attempt to open up connection with device
  for(unsigned int x=0; x<deviceCount; x++) {

    // attempt to open up connection
    if(!device.open(x)) {
      continue;
    }
  }

  // report on connection status
  if(!device.isOpen()) {
    error = "unable to communicate with any of the devices";
    return;
  }
*/
  // create callback function thread
  callbackThread = new boost::thread(falconCallback, this);
}

Falcon::~Falcon() {

  // close the thread
  callbackThread->join();

  // close the falcon communication
  device.close();

}

bool Falcon::hasError() {
  return !error.empty();
}

string Falcon::getError() {
  return error;
}

void falconCallback(Falcon *falcon) {
}
