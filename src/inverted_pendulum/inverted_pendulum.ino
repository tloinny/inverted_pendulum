#include <MotorDriver.h>
#include <seeed_pwm.h>
#include <AS5600.h>
#include <easy_inverted_pendulum.h>
#include <Wire.h>
#include <Ultrasonic.h>
#include <Math.h>

#define SWING_UP_RANG 170
#define WITHIN_PID_SCOPE (Angle_encoder > Inverted_pendulum_controller.GetAngleSetPoint() - SWING_UP_RANG / 2 && Angle_encoder < Inverted_pendulum_controller.GetAngleSetPoint() + SWING_UP_RANG / 2)
#define WITHIN_SWING_SCOPE (Angle_encoder <= Inverted_pendulum_controller.GetAngleSetPoint() - SWING_UP_RANG / 2 || Angle_encoder >= Inverted_pendulum_controller.GetAngleSetPoint() + SWING_UP_RANG / 2)
#define offset 0

#define A_kp 46
#define A_kd 4000

#define P_kp 1
#define P_kd 840

const long A_sample_time = 0;
const long P_sample_time = 0;
const int A_limit = 255;
const int P_limit = 255;

MotorDriver motor;

Ultrasonic ultrasonic(4);

AMS_5600 ams_5600;	/* angle encoder */
inverted_pendulum Inverted_pendulum_controller(A_kp, A_kd, P_kp, P_kd, A_sample_time, P_sample_time, A_limit, P_limit);	/* inverted pendulum pid controller 35, 90, 5.5, 50, 200, 100 */

float Angle_encoder = 0;
float Position_encoder = 0;

float output = 0;
float throttle = 0;

long time_log = millis();

/**
 *@Function: convertScaledAngleToDegrees
 * In: angle data from AMS_5600::getScaledAngle
 * Out: human readable degrees as float
 * Description: takes the scaled angle and calculates
 * float value in degrees.
 */
float convertScaledAngleToDegrees(word newAngle)
{
  word startPos = ams_5600.getStartPosition();
  word endPos = ams_5600.getEndPosition();
  word maxAngle = ams_5600.getMaxAngle();

  float multipler = 0;

  /* max angle and end position are mutually exclusive*/
  if(maxAngle >0)
  {
    if(startPos == 0)
      multipler = (maxAngle*0.0878)/4096;
    else  /*startPos is set to something*/
      multipler = ((maxAngle*0.0878)-(startPos * 0.0878))/4096;
  }
  else
  {
    if((startPos == 0) && (endPos == 0))
      multipler = 0.0878;
    else if ((startPos > 0 ) && (endPos == 0))
      multipler = ((360 * 0.0878) - (startPos * 0.0878)) / 4096;
    else if ((startPos == 0 ) && (endPos > 0))
      multipler = (endPos*0.0878) / 4096;
    else if ((startPos > 0 ) && (endPos > 0))
      multipler = ((endPos*0.0878)-(startPos * 0.0878))/ 4096;
  }
  return (newAngle * multipler);
}

void setup()
{
	Serial.begin(115200);
	Wire.begin();

	ams_5600.setStartPosition(word(0/0.087));
	ams_5600.setEndPosition(word(356/0.087));	/* set the range of as5600 */
	Inverted_pendulum_controller.SetAngleSetPoint(178.31);
	Inverted_pendulum_controller.SetPositionSetPoint(20.5*10);

	motor.begin();

	Serial.println("Invered Pendulum");
	Serial.print("A_KP:");
	Serial.print(A_kp);
	Serial.print(" ");
	Serial.print("A_KD:");
	Serial.print(A_kd);
	Serial.print(" ");
	Serial.print("P_KP:");
	Serial.print(P_kp);
	Serial.print(" ");
	Serial.print("P_KD:");
	Serial.println(P_kd);
	Serial.println("-----------------------------");	/* print the information */
}

void loop()
{
	/* You can uncomment this part and use serialplot to see the angle feebback curve */
	
	Serial.print(Inverted_pendulum_controller.GetAngleSetPoint());
	Serial.print(",");
	Serial.println(Angle_encoder);
	

	Angle_encoder = convertScaledAngleToDegrees(ams_5600.getScaledAngle());
	ultrasonic.MeasureInCentimeters();
	Position_encoder = round(ultrasonic.RangeInCentimeters*10);

	if (WITHIN_PID_SCOPE && Position_encoder >= 30 && Position_encoder <= 400)
	{
		output = Inverted_pendulum_controller.InvertedPendulumUpdate(Angle_encoder, Position_encoder);
		if (output > 0)
		{
			throttle = output + offset;
				if(throttle > 255) throttle = 255;
			motor.speed(0, throttle);
		}else if (output < 0)
		{
			output = -1*output;
			throttle = output + offset;
				if(throttle > 255) throttle = 255;
			motor.speed(0, -1*throttle);
		}else
		{
			motor.brake(0);
		}
	}else
	{
		motor.brake(0);
	}
}
