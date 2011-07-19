#ifndef __COAX_ONBOARD_CONTROL__
#define __COAX_ONBOARD_CONTROL__

class CoaXOnboardControl
{
public:
  CoaXOnboardControl() : motor_upper(0), motor_lower(0),
                         servo_upper(0), servo_lower(0) { }
  ~CoaXOnboardControl() {}

  void SetCommands(double motor1, double motor2,
                   double servo1, double servo2)
  {
    motor_upper = motor1;
    motor_lower = motor2;

    servo_upper = servo1;
    servo_lower = servo2;
  }

  void GetControls(double& motor1, double& motor2,
                   double& servo1, double& servo2)
  {
    motor1 = motor_upper;
    motor2 = motor_lower;

    servo1 = servo_upper;
    servo2 = servo_lower;
  }

private:
  double motor_upper, motor_lower;
  double servo_upper, servo_lower;
};

#endif
