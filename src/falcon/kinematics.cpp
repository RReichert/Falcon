#include "falcon/kinematics.hpp"

void Kinematics::decodeTheta(const boost::array<int, 3> (&encodedTheta), boost::array<double, 3> (&theta)) {
  for(int x=0; x<3; x++) {
    theta[x] = decodeTheta(encodedTheta[x]);
  } 
}

void Kinematics::encodeTorque(const boost::array<double, 3> (&omega), const boost::array<double, 3> (&torque), boost::array<int, 3> (&encodedTorque)) {
  for(int x=0; x<3; x++) {
    encodedTorque[x] = encodeTorque(omega[x], torque[x]);
  }
}

void Kinematics::d_dt(const boost::array<double, 3> (&currentValue), const boost::array<double, 3> (&prevValue), double dt, boost::array<double, 3> (&dValue_dt)) {
  for(int x=0; x<3; x++) {
    dValue_dt[x] = (currentValue[x] - prevValue[x]) / (dt);
  }
}

bool Kinematics::inverse_kinematics(const boost::array<double, 3> (&position), boost::array<double, 3> (&theta)) {

  // local variables
  bool success;
  boost::array<double, 9> thetas;

  // call out to inverse kinematics
  success = inverse_kinematics(position, thetas);

  // if the position was valid and thetas were found, copy across the base thetas
  if(success) {
    theta[0] = thetas[0];
    theta[1] = thetas[3];
    theta[2] = thetas[6];
  }

  return success;
}

bool Kinematics::inverse_kinematics(const boost::array<double, 3> (&position), boost::array<double, 9> (&thetas)) {

  // function variables
  double theta1, theta2, theta3;
  double rx, ry, rz;
  double Rx, Ry, Rz;
  double X, Y, Z;
  double A, B, C;
  double L;
  double phi[3] = {phi1, phi2, phi3};
  double config1, config2;
  boost::array<double, 9> tempThetas;

  // obtain each position element
  rx = position[0];
  ry = position[1];
  rz = position[2];

  // loop through each arm and calculate the arm theta values
  bool error = false;
  for(int i=0; i<3; i++) {

    Rx = rx*cos(phi[i]) + ry*sin(phi[i]);
    Ry = -rx*sin(phi[i]) + ry*cos(phi[i]);
    Rz = rz;

    theta3 = asin((Rx+f-s) / b);
    L = d + b*cos(theta3) + e;

    X = Ry + c - r;
    Y = Rz;
    Z = (a*a +r*r + c*c + Ry*Ry + Rz*Rz + 2*Ry*c -2*Ry*r - 2*r*c - L*L) / (2*a);

    A = -(X+Z);
    B = 2*Y;
    C = X-Z;

    config1 = 2*atan( (-B + sqrt(B*B-4*A*C))  / (2*A) );
    config2 = 2*atan( (-B - sqrt(B*B-4*A*C))  / (2*A) );
    
    if(config1 >= min_theta && config1 <= max_theta) {
      theta1 = config1;
    } else if(config2 >= min_theta && config2 <= max_theta) {
      theta1 = config2;
    } else {
      error = true;
      break;
    }

    theta2 = asin( -(Ry + c - r - a*cos(theta1)) / L );

    tempThetas[3*i]   = theta1;
    tempThetas[3*i+1] = theta2;
    tempThetas[3*i+2] = theta3;
  }

  // if there was not error, transfer over from temp theta to theta
  if(!error) {
    for(int x=0; x<9; x++){
      thetas[x] = tempThetas[x];
    }
  }

  // indicate status
  return !error;
}

bool Kinematics::forward_kinematics(const boost::array<double, 3> (&theta), const boost::array<double, 3> (&positionEstimate), boost::array<double, 3> (&position)) {

  // declare function variables
  bool error = false;
  boost::numeric::ublas::matrix<double> Ok(9,1);              // current configuration estimation
  boost::numeric::ublas::matrix<double> Okp1(9,1);            // next configuration estimation
  boost::numeric::ublas::matrix<double> temp(9,1);            // next configuration estimation
  boost::numeric::ublas::matrix<double> fn(9,1);              // newton raphson F vector
  boost::numeric::ublas::matrix<double> fn_dash(9,9);	        // newton raphson F' matrix
  boost::numeric::ublas::matrix<double> fn_dash_inverse(9,9);	// newton raphson inv(F') matrix

  boost::array<double, 9> tempThetas;	  // local estimate thetas
  boost::array<double, 3> tempPosition;	// local estimate position

  // transfer estimate position to our local position vector
  for(int x=0; x<3; x++) {
    tempPosition[x] = position[x];
  }

  // obtain the estimated thetas configuration
  error = inverse_kinematics(tempPosition, tempThetas);

  // report if estimate point is outside the workspace
  if(error) {
    return error;
  }

  // setup first estimate values
  Okp1(0,0) = tempPosition[0];
  Okp1(1,0) = tempThetas[1];
  Okp1(2,0) = tempThetas[2];
  Okp1(3,0) = tempPosition[1];
  Okp1(4,0) = tempThetas[4];
  Okp1(5,0) = tempThetas[5];
  Okp1(6,0) = tempPosition[2];
  Okp1(7,0) = tempThetas[7];
  Okp1(8,0) = tempThetas[8];

  // perform estimation algorithm
  unsigned int iterations = 0;
  do
  {
    // stop algorithm if it has taken too long
    if(iterations >= max_iterations) {
      break;
    } else {
      iterations++;
    }

    // update previous estimate with current estimate
    Ok = Okp1;

    // setup the position vector
    tempPosition[0] = Ok(0,0);
    tempPosition[0] = Ok(3,0);
    tempPosition[0] = Ok(6,0);

    // setup the thetas vector (grab the base angles and the estimate angles)
    tempThetas[0] = theta[0];
    tempThetas[1] = Ok(1,0);
    tempThetas[2] = Ok(2,0);
    tempThetas[3] = theta[1];
    tempThetas[4] = Ok(4,0);
    tempThetas[5] = Ok(5,0);
    tempThetas[6] = theta[2];
    tempThetas[7] = Ok(7,0);
    tempThetas[8] = Ok(8,0);

    // calculate the F and F' newton raphons vector/matrix
    constraint_matrix(tempThetas, tempPosition, fn);
    constraint_matrix_dash(tempThetas, fn_dash);

    // calculate inv(F')*F using LU Decomposition
    inverse_matrix(fn_dash, fn_dash_inverse);
    temp = boost::numeric::ublas::prod(fn_dash_inverse, fn);

    // calculate Okp1 = Ok - inv(F')*F
    Okp1 = Ok - temp;
  }
  while(max_error(Okp1,Ok) > tolerance);

  // if the algorithm was unable to find an adaquete solution
  if(iterations == max_iterations) {
    return false;
  }

  // transfer solutions to
  position[0] = Okp1(0,0);
  position[1] = Okp1(3,0);
  position[2] = Okp1(6,0);

  // indicate successful result
  return true;
}

double Kinematics::max_error(const boost::numeric::ublas::matrix<double> (&a), const boost::numeric::ublas::matrix<double> (&b)) {
  assert(a.size1() != 0);
  assert(a.size1() ==  b.size1());
  assert(a.size2() ==  b.size2());

  double error = std::abs( a(0,0) - b(0,0) );
  for(unsigned int x=0; x<a.size1(); x++) {
    for(unsigned int y=0; y<a.size2(); y++) {
      error = std::abs( std::max(a(x,y), b(x,y)) );
    }
  }

  return error;
}

bool Kinematics::inverse_matrix(const boost::numeric::ublas::matrix<double> (&m), boost::numeric::ublas::matrix<double> (&inv_m)) {

  assert(m.size1() != 0);
  assert(m.size1() ==  m.size2());
  assert(m.size1() ==  inv_m.size1());
  assert(m.size2() ==  inv_m.size2());

  // local variables
  boost::numeric::ublas::matrix<double> a(m);
  boost::numeric::ublas::permutation_matrix<std::size_t> pm(a.size1());

  // run factorization and check that it's not singular
  if( boost::numeric::ublas::lu_factorize(a,pm) != 0 ) {
    return false;
  }

  // solve the inverse matrix
  inv_m.assign( boost::numeric::ublas::identity_matrix<double>(a.size1()) );
  boost::numeric::ublas::lu_substitute(a, pm, inv_m);

  // return successful inverse
  return true;
}

void Kinematics::constraint_matrix(const boost::array<double, 9> (&thetas), const boost::array<double, 3> (&position), boost::numeric::ublas::matrix<double> (&fn)) {
  fn.clear();
  fn(0,0) = -cos(phi1)*(-f+s+b*sin(thetas[2]))+cos(phi2)*(-f+s+b*sin(thetas[5]))-sin(phi1)*(c-r+sin(thetas[1])*(d+e+b*cos(thetas[2]))-a*cos(thetas[0]))+sin(phi2)*(c-r+sin(thetas[4])*(d+e+b*cos(thetas[5]))-a*cos(thetas[3]));
  fn(1,0) = cos(phi1)*(c-r+sin(thetas[1])*(d+e+b*cos(thetas[2]))-a*cos(thetas[0]))-cos(phi2)*(c-r+sin(thetas[4])*(d+e+b*cos(thetas[5]))-a*cos(thetas[3]))-sin(phi1)*(-f+s+b*sin(thetas[2]))+sin(phi2)*(-f+s+b*sin(thetas[5]));
  fn(2,0) = -a*sin(thetas[0])+a*sin(thetas[3])-cos(thetas[1])*(d+e+b*cos(thetas[2]))+cos(thetas[4])*(d+e+b*cos(thetas[5]));
  fn(3,0) = -cos(phi1)*(-f+s+b*sin(thetas[2]))+cos(phi3)*(-f+s+b*sin(thetas[8]))-sin(phi1)*(c-r+sin(thetas[1])*(d+e+b*cos(thetas[2]))-a*cos(thetas[0]))+sin(phi3)*(c-r+sin(thetas[7])*(d+e+b*cos(thetas[8]))-a*cos(thetas[6]));
  fn(4,0) = cos(phi1)*(c-r+sin(thetas[1])*(d+e+b*cos(thetas[2]))-a*cos(thetas[0]))-cos(phi3)*(c-r+sin(thetas[7])*(d+e+b*cos(thetas[8]))-a*cos(thetas[6]))-sin(phi1)*(-f+s+b*sin(thetas[2]))+sin(phi3)*(-f+s+b*sin(thetas[8]));
  fn(5,0) = -a*sin(thetas[0])+a*sin(thetas[6])-cos(thetas[1])*(d+e+b*cos(thetas[2]))+cos(thetas[7])*(d+e+b*cos(thetas[8]));
  fn(6,0) = -position[0]+cos(phi1)*(-f+s+b*sin(thetas[2]))+sin(phi1)*(c-r+sin(thetas[1])*(d+e+b*cos(thetas[2]))-a*cos(thetas[0]));
  fn(7,0) = -position[1]-cos(phi1)*(c-r+sin(thetas[1])*(d+e+b*cos(thetas[2]))-a*cos(thetas[0]))+sin(phi1)*(-f+s+b*sin(thetas[2]));
  fn(8,0) = -position[2]+a*sin(thetas[0])+cos(thetas[1])*(d+e+b*cos(thetas[2]));
}

void Kinematics::constraint_matrix_dash(const boost::array<double, 9> (&thetas), boost::numeric::ublas::matrix<double> (&fn_dash)) {
  fn_dash.clear();
  fn_dash(0,3) = -cos(thetas[1])*sin(phi1)*(d+e+b*cos(thetas[2]));
  fn_dash(0,4) = -b*cos(phi1)*cos(thetas[2])+b*sin(phi1)*sin(thetas[1])*sin(thetas[2]);
  fn_dash(0,5) = cos(thetas[4])*sin(phi2)*(d+e+b*cos(thetas[5]));
  fn_dash(0,6) = b*cos(phi2)*cos(thetas[5])-b*sin(phi2)*sin(thetas[4])*sin(thetas[5]);
  fn_dash(1,3) = cos(phi1)*cos(thetas[1])*(d+e+b*cos(thetas[2]));
  fn_dash(1,4) = -b*cos(thetas[2])*sin(phi1)-b*cos(phi1)*sin(thetas[1])*sin(thetas[2]);
  fn_dash(1,5) = -cos(phi2)*cos(thetas[4])*(d+e+b*cos(thetas[5]));
  fn_dash(1,6) = b*cos(thetas[5])*sin(phi2)+b*cos(phi2)*sin(thetas[4])*sin(thetas[5]);
  fn_dash(2,3) = sin(thetas[1])*(d+e+b*cos(thetas[2]));
  fn_dash(2,4) = b*cos(thetas[1])*sin(thetas[2]);
  fn_dash(2,5) = -sin(thetas[4])*(d+e+b*cos(thetas[5]));
  fn_dash(2,6) = -b*cos(thetas[4])*sin(thetas[5]);
  fn_dash(3,3) = -cos(thetas[1])*sin(phi1)*(d+e+b*cos(thetas[2]));
  fn_dash(3,4) = -b*cos(phi1)*cos(thetas[2])+b*sin(phi1)*sin(thetas[1])*sin(thetas[2]);
  fn_dash(3,7) = cos(thetas[7])*sin(phi3)*(d+e+b*cos(thetas[8]));
  fn_dash(3,8) = b*cos(phi3)*cos(thetas[8])-b*sin(phi3)*sin(thetas[7])*sin(thetas[8]);
  fn_dash(4,3) = cos(phi1)*cos(thetas[1])*(d+e+b*cos(thetas[2]));
  fn_dash(4,4) = -b*cos(thetas[2])*sin(phi1)-b*cos(phi1)*sin(thetas[1])*sin(thetas[2]);
  fn_dash(4,7) = -cos(phi3)*cos(thetas[7])*(d+e+b*cos(thetas[8]));
  fn_dash(4,8) = b*cos(thetas[8])*sin(phi3)+b*cos(phi3)*sin(thetas[7])*sin(thetas[8]);
  fn_dash(5,3) = sin(thetas[1])*(d+e+b*cos(thetas[2]));
  fn_dash(5,4) = b*cos(thetas[1])*sin(thetas[2]);
  fn_dash(5,7) = -sin(thetas[7])*(d+e+b*cos(thetas[8]));
  fn_dash(5,8) = -b*cos(thetas[7])*sin(thetas[8]);
  fn_dash(6,0) = -1.0;
  fn_dash(6,3) = cos(thetas[1])*sin(phi1)*(d+e+b*cos(thetas[2]));
  fn_dash(6,4) = b*cos(phi1)*cos(thetas[2])-b*sin(phi1)*sin(thetas[1])*sin(thetas[2]);
  fn_dash(7,1) = -1.0;
  fn_dash(7,3) = -cos(phi1)*cos(thetas[1])*(d+e+b*cos(thetas[2]));
  fn_dash(7,4) = b*cos(thetas[2])*sin(phi1)+b*cos(phi1)*sin(thetas[1])*sin(thetas[2]);
  fn_dash(8,2) = -1.0;
  fn_dash(8,3) = -sin(thetas[1])*(d+e+b*cos(thetas[2]));
  fn_dash(8,4) = -b*cos(thetas[1])*sin(thetas[2]);
}
