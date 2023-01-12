#include "main.h"
#include "config.hpp"
#include "odom.hpp"
#include "robot.hpp"
#include "movement.hpp"
#include "PID.hpp"
#include "api.h"
#include "auton.hpp"
#include "pros/apix.h"
#include "display/lvgl.h"
#include "initialize.hpp"
#include "vision.hpp"

void lvgldisplay()
{
	
	char buffer[100];
	sprintf(buffer, "X: %f\nY: %f\nT: %f\nPOT: %d\nGRABBED: %d\nCLAMPED: %d\nfwd: %d\n intakeSpeed: %d", pos_x, pos_y, theta * 180 / M_PI, liftPot.get_position(), grabbed(),clamped(),fwdRotationValue(),intakeSpeed);
	lv_label_set_text(OdomDisplay, buffer);
	if(leftB.get_brake_mode() == pros::E_MOTOR_BRAKE_COAST)
		sprintf(buffer, "COAST\nLEFTTEMP: %f\nRIGHTTEMP: %f\nLIFTTEMP: %f\n LeftWatts: %f\n RightWatts: %f\nINTAKETEMP: %f\n intakeTorq: %f", (leftM.get_temperature() + leftF.get_temperature() + leftB.get_temperature())/3,  (rightM.get_temperature() + rightF.get_temperature() + rightB.get_temperature()) / 3, (lift.get_temperature()), (leftM.get_power() + leftF.get_power() + leftB.get_power()) / 3, (rightM.get_power() + rightF.get_power() + rightB.get_power()) / 3,intake.get_temperature(),intake.get_torque());
	else if(leftB.get_brake_mode() == pros::E_MOTOR_BRAKE_HOLD)
		sprintf(buffer, "HOLD\nLEFTTEMP: %f\nRIGHTTEMP: %f\nLIFTTEMP: %f\n LeftWatts: %f\n RightWatts: %f\nINTAKETEMP: %f\n intakeTorq: %f", (leftM.get_temperature() + leftF.get_temperature() + leftB.get_temperature())/3,  (rightM.get_temperature() + rightF.get_temperature() + rightB.get_temperature()) / 3, (lift.get_temperature()), (leftM.get_power() + leftF.get_power() + leftB.get_power()) / 3, (rightM.get_power() + rightF.get_power() + rightB.get_power()) / 3,intake.get_temperature(),intake.get_torque());

	lv_label_set_text(mainDisplay, buffer);
}

void autonDisplay()
{
	char buffer[100];
	sprintf(buffer, "AutonDisplay\nX: %f\nY: %f\nT: %f\n", pos_x, pos_y, theta * 180 / M_PI);
	lv_label_set_text(AutonDisplay, buffer);
}

void opcontrol()
{
	pros::vision_signature_s_t YELLOW_SIG = pros::Vision::signature_from_utility(1, 1079, 1425, 1252, -3661, -3347, -3504, 5.900, 0);
	vision.set_signature(1, &YELLOW_SIG);
	if (isnanf(pos_x))
	{
		pos_x = 0;
	}
	if (isnanf(pos_y))
	{
		pos_y = 0;
	}
	// updateOdom.remove();
	// updateTime.remove();
	while (true)
	{
		setBreakTypeCoast();
		drive();
		liftOP();
		toggleLift();
		pistons();
		intakeOP();
		// transmissionOP();
		lvgldisplay();
		visionDisplay(); // autonDisplay();
		// climbToggle();
		// odom(); 

		// printf("LEFTWATTS: %f, RIGHTWATTS: %f\n",(leftM.get_power() + leftF.get_power() + leftB.get_power()) / 3, (rightM.get_power() + rightF.get_power() + rightB.get_power()) / 3);
		twoSensorOdom();
		// clampAuton();
		// frontGrabberAuton();

		// master.print(0, 0, "W: %f", (leftM.get_power() + leftF.get_power() + leftB.get_power() + rightM.get_power() + rightF.get_power() + rightB.get_power()) / 6);
		counter++;
		pros::delay(10);
	}
}
