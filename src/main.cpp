  #include "main.h"

void initialize() {
	Motor FR(FRport, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
	Motor BR(BRport, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
	Motor FL(FLport, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
	Motor BL(BLport, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
	Motor RollerT(RollerTport, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
	Motor IntakeR(IntakeRport, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
	Motor IntakeL(IntakeLport, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
  Motor RollerB(RollerBport, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
  ADILineSensor Topsensor(Topsensorport);
  ADILineSensor Bottomsensor(Bottomsensorport);
}

void basemove(double distance,double Cutoff) {
  Motor BL(BLport);
  Motor BR(BRport);
  Motor FL(FLport);
  Motor FR(FRport);
  double EncL = BL.get_position();
  double EncR = BR.get_position();

  int start = millis();

  double targencL = distance;
  double targencR = -distance;

  double ErrorL = targencL - EncL;
  double ErrorR = targencR - EncR;

  double PrevErrorL = 0;
  double PrevErrorR = 0;

  double kp = 0;
  double kd = 0;

  double DeltaErrorL = ErrorL - PrevErrorL;
  double DeltaErrorR = ErrorR - PrevErrorR;

  double PowerL = 0;
  double PowerR = 0;

  double PrevPowerL = 0;
  double PrevPowerR = 0;

while ((fabs(ErrorL) > 10 || fabs(ErrorR)>10) && (millis()-start < Cutoff)){
    double PowerL = ErrorL*kp + DeltaErrorL*kd;
    double PowerR = ErrorR*kp + DeltaErrorR*kd;

    PrevErrorL = ErrorL;
    PrevErrorR = ErrorR;

    //ramping
  if(PowerL > PrevPowerL +5 || PowerL < PrevPowerL-5){
    if(PowerL > PrevPowerL+5 ){
      PowerL = PrevPowerL+5;
    }
    else{
      PowerL = PrevPowerL-5;
    }
  }
  if(PowerR > PrevPowerR +5 || PowerR < PrevPowerR-5){
    if(PowerR > PrevPowerR+5 ){
      PowerR = PrevPowerR+5;
    }
    else{
      PowerR = PrevPowerR-5;
    }
  }

    FL.move(PowerL);
    FR.move(PowerR);
    BR.move(PowerR);
    BL.move(PowerL);

    PrevPowerL = PowerL;
    PrevPowerR = PowerR;
    printf("Errors: %.2f, %.2f\n", ErrorL, ErrorR);
    delay(5);
  }
    FL.move(0);
    FR.move(0);
    BL.move(0);
    BR.move(0);
    BR.tare_position();
    BL.tare_position();


}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  Motor FL(FLport);
  Motor BL(BLport);
  Motor FR(FRport);
  Motor BR(BRport);
  Motor IntakeR(IntakeRport);
  Motor IntakeL(IntakeLport);
  Motor RollerB(RollerBport);
  Motor RollerT(RollerTport);
  ADILineSensor Topsensor(Topsensorport);
  ADILineSensor Bottomsensor(Bottomsensorport);

}

/** or control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	Motor FL(FLport);
	Motor BL(BLport);
	Motor FR(FRport);
	Motor BR(BRport);
	Motor IntakeR(IntakeRport);
	Motor IntakeL(IntakeLport);
	Motor RollerB(RollerBport);
	Motor RollerT(RollerTport);
  ADILineSensor Topsensor(Topsensorport);
  ADILineSensor Bottomsensor(Bottomsensorport);


	Controller master(E_CONTROLLER_MASTER);
  bool arcade = true;
	while (true) {
    if(master.get_digital_new_press(DIGITAL_X)){
      arcade = !arcade;
    }
    if(arcade){
      double power = master.get_analog(ANALOG_LEFT_Y);
      double turn = master.get_analog(ANALOG_RIGHT_X);
      double left = power + turn;
      double right = power - turn;
      FL.move(left);
      BL.move(left);
      BR.move(right);
      FR.move(right);
    } else{
      double left = master.get_analog(ANALOG_LEFT_Y);
      double right = master.get_analog(ANALOG_RIGHT_Y);
      FL.move(left);
		  BL.move(left);
		  BR.move(right);
		  FR.move(right);
    }

		if (master.get_digital(DIGITAL_L2)) {
			IntakeR.move(-127);
			IntakeL.move(127);
		}
		else if (master.get_digital(DIGITAL_L1)) {
			IntakeR.move(127);
			IntakeL.move(-127);
		}
		else {
			IntakeR.move(0);
			IntakeL.move(0);
		}


	if(Topsensor.get_value()>2700){
      RollerB.move(-127);
    }
  else if(Topsensor.get_value()<2700){
    if(Bottomsensor.get_value()<2700){
      RollerB.move(0);
    }
    else{
      RollerB.move(-127);
    }
  }
  else{
    RollerB.move(0);
  }

  if(master.get_digital(DIGITAL_R1)){
    RollerT.move(127);
    RollerB.move(-127);
  }
  else if (master.get_digital(DIGITAL_R2)){
    RollerT.move(-127);
    RollerB.move(127);
  }
  else{
   RollerT.move(0);

 }



		delay(5);
	}
}
