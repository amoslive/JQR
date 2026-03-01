// File:          waistcar_jump_cpp.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/GPS.hpp>
#include "car_jump.h"


using namespace webots;


int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int baseStep = (int)robot->getBasicTimeStep();
  int timestep = 2*baseStep;

  Motor *rear_motor = robot->getMotor("rear_wheel_motor");
  Motor *waist_motor = robot->getMotor("waist_motor");
  InertialUnit *imu = robot->getInertialUnit("imu");
  imu->enable(baseStep);
  PositionSensor *waist_sensor = robot->getPositionSensor("waist_sensor");
  waist_sensor->enable(baseStep);
  PositionSensor *rear_sensor = robot->getPositionSensor("rear_wheel_sensor");
  rear_sensor->enable(baseStep);
  GPS *gps = robot->getGPS("gps");
  gps->enable(baseStep);

  car_jump car;
  
  car.orientation = imu->getRollPitchYaw();
  car.waist_pos = waist_sensor->getValue();
  car.wheel_pos = rear_sensor->getValue();
  car.position = gps->getValues(); 
  car.initial(timestep);

  std::cout << "hello!" << std::endl;

  while (robot->step(timestep) != -1) {

    car.t = car.t + 0.001*timestep;
    car.orientation = imu->getRollPitchYaw();
    car.waist_pos = waist_sensor->getValue();
    car.wheel_pos = rear_sensor->getValue();
    car.position = gps->getValues();

    car.run();


    rear_motor->setTorque(car.fr);
    waist_motor->setTorque(car.fb);

    std::cout << "t:  "<< car.t <<"  pitch_pos:  "<<car.pitch_pos<< "  fr:  "<< car.fr << "   rollpos:  "<< car.roll_pos << std::endl; 
  };



  delete robot;
  return 0;
}
