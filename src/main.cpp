#include <cstdlib>
#include <iostream>
#include <boost/array.hpp>
#include <boost/date_time.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

#include "falcon/falcon.hpp"
#include "falcon/kinematics.hpp"
#include "falcon/controller/test.hpp"

void init_debug() {
  boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::debug);
}

int main()
{
  init_debug();

  Falcon<Test_Controller> falcon;

  if(!falcon.isInit()) {
    BOOST_LOG_TRIVIAL(debug) << "falcon error: " << falcon.getError();
    return EXIT_FAILURE;
  }

  falcon.start();

  boost::this_thread::sleep(boost::posix_time::seconds(100));

  return EXIT_SUCCESS;
}