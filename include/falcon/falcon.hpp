#pragma once
#include "falcon/kinematics.hpp"
#include "falcon/controller/controller.hpp"

#include <iostream>
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/type_traits.hpp>
#include <boost/static_assert.hpp>
#include <falcon/core/FalconDevice.h>
#include <falcon/firmware/FalconFirmwareNovintSDK.h>
#include <falcon/util/FalconFirmwareBinaryNvent.h>

using namespace std;
using namespace libnifalcon;

template<class T=Controller>
class Falcon {

  // ASSERT THAT THE CLASS BEING PASSED IS A CHILD OF CONTROLLER CLASS
  BOOST_STATIC_ASSERT((boost::is_base_of<Controller, T>::type::value));

  private:

    // state
    bool running;
    bool initialized;

    // initialize time
    boost::posix_time::ptime initTime;

    // device
    FalconDevice device;
    Kinematics kinematics;
    boost::shared_ptr<FalconFirmware> firmware;

    // controller
    Controller *controller;

    // falcon-controller shared resource
    boost::mutex motionMutex;
    boost::array<double, 3> theta;
    boost::array<double, 3> omega;
    boost::posix_time::ptime time;

    boost::mutex desiredMutex;
    boost::array<double, 3> desiredTheta;

    // callback thread
    boost::thread *callbackThread;

    // error message
    string error;

  public:

    // CONSTRUCTOR
    Falcon();

    // DECONSTRUCTOR
    ~Falcon();

    // initializer
    bool init();

    // un-initialize
    void uninit();

    // initialized?
    bool isInit();

    // starts controller
    void start();

    // stops controller
    void stop();

    // running?
    bool isRunning();

    // error methods
    bool hasError();
    string getError();

    // public sharing resource 
    bool setDesiredPosition(boost::array<double, 3> (&desiredPosition));
    bool getMotion(boost::posix_time::ptime (&time), boost::array<double, 3> (&theta), boost::array<double, 3> (&omega));

    // NOTE: for each of the methods above, if the system is not initialized
    //       or there is an error, false is returned. only exception is
    //       setDesiredPosition, where if the position is outside the workspace
    //       the value will not be set and function will return false. if all 
    //       is well, true is returned

    // CALLBACK FUNCTION
    void operator() (); 

  private:

    // private sharing resource
    void getDesiredTheta(boost::array<double, 3> (&desiredTheta));
    void setMotion(boost::posix_time::ptime (&time), boost::array<double, 3> (&theta), boost::array<double, 3> (&omega));

};

template<class T>
void Falcon<T>::operator() () {

  // function variables
  double dt;

  boost::array<double, 3> theta;
  boost::array<double, 3> omega;
  boost::array<double, 3> torque;
  boost::posix_time::ptime time;

  boost::array<double, 3> prevTheta;
  boost::array<double, 3> prevOmega;
  boost::posix_time::ptime prevTime;

  boost::array<int, 3> encodedTheta;
  boost::array<int, 3> encodedTorque;

  // while device is initialized
  while(initialized) {


    // MAIN DEVICE LOOP 
    if(!firmware->runIOLoop()) {
      continue;
    }

    // IDENTIFY THE FOLLOWING SECTION AS NONE INTERRUPTABLE
    {
      // interruption marker
      boost::this_thread::disable_interruption iPoint; 

      // capture the previous motion values
      getMotion(prevTime, prevTheta, prevOmega);

      // capture the current motion values
      encodedTheta = firmware->getEncoderValues();
      kinematics.decodeTheta(encodedTheta, theta); 
      time = boost::posix_time::microsec_clock::local_time();

      // obtain the time difference between current and previous 
      dt = ((double) (time - prevTime).total_microseconds()) * 1.0e-6;

      // calculate omega
      kinematics.d_dt(theta, prevTheta, dt, omega);

      // save the current motion values
      setMotion(time, theta, omega);

      // check if you need to run controller 
      if(!running) {
        continue;
      }

      // obtain desired theta 
      getDesiredTheta(desiredTheta);

      // run falcon controller 
      controller->getTorque(theta, desiredTheta, torque);

      // convert torque to motor voltages:
      kinematics.encodeTorque(omega, torque, encodedTorque);

      // set desired torque
      firmware->setForces(encodedTorque);
    }
  }

}

template<class T>
Falcon<T>::Falcon() :
  running(false),
  initialized(false),
  controller(NULL),
  callbackThread(NULL) {

  // set default desired theta value
  desiredTheta = {{0,0,0.11}};

  // set/obtain firmware
  device.setFalconFirmware<FalconFirmwareNovintSDK>();
  firmware = device.getFalconFirmware();

  // initialize device
  init();
}

template<class T>
Falcon<T>::~Falcon() {
  uninit();
}

template<class T>
bool Falcon<T>::init() {

  // uninitialize device
  if(initialized) {
    uninit();
  }

  // reset theta and omega
  theta = {{0,0,0}};
  omega = {{0,0,0}};

  // clear any error message
  error.clear();

  try{

    // check how many devices are connected
    unsigned int deviceCount;
    device.getDeviceCount(deviceCount);

    // error if not devices were found 
    if(deviceCount == 0) {
      throw "no device found";
    }

    // attempt to open up connection with device
    for(unsigned int x=0; x<deviceCount; x++) {
      if(!device.open(x)) {
        continue;
      }
    }

    // report failure to connect with device
    if(!device.isOpen()) {
      throw "unable to communicate with any of the devices";
    }

    // check firmware is initialized
    bool firmwareLoaded = device.isFirmwareLoaded();
    if(!firmwareLoaded) {

      // firmware variables
      bool skip_checksum = true;
      long firmware_size = NOVINT_FALCON_NVENT_FIRMWARE_SIZE;
      uint8_t* firmware_block = const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE);

      // attempt to load firmware
      for(int x=0; x<10; x++) {
        if(device.getFalconFirmware()->loadFirmware(skip_checksum, firmware_size, firmware_block)) {
          firmwareLoaded = true;
          continue;
        }
      }
    }

    // report if firmware was not loaded 
    if(!device.isFirmwareLoaded()) {
      throw "unable to load firmware";
    }

    // attempt to communicate with falcon
    bool working = false;
    for(int x=0; x<10; x++) {
      if(device.runIOLoop(FalconDevice::FALCON_LOOP_FIRMWARE)) {
        working = true;
      }
    }

    // report if communication is down
    if(!working) {
      throw "unable to run IO loop";
    }

    // obtain theta
    boost::array<int, 3> encodedTheta = firmware->getEncoderValues();
    kinematics.decodeTheta(encodedTheta, theta);

    // set time 
    initTime = boost::posix_time::microsec_clock::local_time();
    time = initTime;

    // startup controller
    controller = (Controller*) new T();

    // create callback function 
    callbackThread = new boost::thread(boost::ref(*this));

    // check if callback thread was created
    if(!callbackThread) {
      throw "unable to spawn callback thread";
    }

    // flag successful initialization
    initialized = true;

  } catch(char const* msg) {

    // uninitialized device
    uninit();

    // record error message
    error = msg;
  }

  // return initialized results
  return initialized;
}

template<class T>
void Falcon<T>::uninit() {

  // flag as uninitialized
  initialized = false;

  // flag to stop controller
  stop();

  // close the thread
  if(callbackThread) {
    callbackThread->join();
    delete callbackThread;
  }

  // destroy controller
  if(controller) {
    delete controller;
  }

  // close the falcon communication
  device.close();

}

template<class T>
bool Falcon<T>::isInit() {
  return initialized;
}

template<class T>
void Falcon<T>::start() {
  running = true;
}

template<class T>
void Falcon<T>::stop() {
  running = false;
}

template<class T>
bool Falcon<T>::isRunning() {
  return running;
}

template<class T>
bool Falcon<T>::hasError() {
  return !error.empty();
}

template<class T>
string Falcon<T>::getError() {
  return error;
}

template<class T>
void Falcon<T>::getDesiredTheta(boost::array<double, 3> (&desiredTheta)) {
  boost::lock_guard<boost::mutex> lock(desiredMutex);
  desiredTheta = this->desiredTheta;
}

template<class T>
bool Falcon<T>::setDesiredPosition(boost::array<double, 3> (&desiredPosition)) {
  boost::lock_guard<boost::mutex> lock(desiredMutex);
//  return kinematic->getAngles(desiredPosition, desiredTheta);
  return true;
}

template<class T>
bool Falcon<T>::getMotion(boost::posix_time::ptime (&time), boost::array<double, 3> (&theta), boost::array<double, 3> (&omega)) {
  if(!initialized) {
    return false;
  }
  
  boost::lock_guard<boost::mutex> lock(motionMutex);
  time = this->time; 
  theta = this->theta;
  omega = this->omega;
  return true;  
}

template<class T>
void Falcon<T>::setMotion(boost::posix_time::ptime (&time), boost::array<double, 3> (&theta), boost::array<double, 3> (&omega)) {
  boost::lock_guard<boost::mutex> lock(motionMutex);
  this->time = time;
  this->theta = theta;
  this->omega = omega;
}
