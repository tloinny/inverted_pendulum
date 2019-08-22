#include <L298N.h>
#include <AS5600.h>
#include <easy_inverted_pendulum.h>
#include <Wire.h>
#include <Ultrasonic.h>
#include <Math.h>

#define SWING_UP_RANG 120
#define WITHIN_PID_SCOPE (Angle_encoder > Inverted_pendulum_controller.GetAngleSetPoint() - SWING_UP_RANG / 2 && Angle_encoder < Inverted_pendulum_controller.GetAngleSetPoint() + SWING_UP_RANG / 2)
#define WITHIN_SWING_SCOPE (Angle_encoder <= Inverted_pendulum_controller.GetAngleSetPoint() - SWING_UP_RANG / 2 || Angle_encoder >= Inverted_pendulum_controller.GetAngleSetPoint() + SWING_UP_RANG / 2)
#define offset 10

const int EN = 4;
const int BRK = 5;
const int DIR = 6;
const int PWM = 9;

#define A_kp 77.5 * 0.6
#define A_kd 8500 * 0.6

#define P_kp 1.3
#define P_kd 2200*0.6

const long A_sample_time = 0;
const long P_sample_time = 0;
const int A_limit = 255;
const int P_limit = 255;

const int ENA = 5;
const int IN1 = 6;
const int IN2 = 7;
const int IN3 = 9;
const int IN4 = 10;
const int ENB = 11;  /* only one motor */

L298N DCMotor(ENA,IN1,IN2,IN3,IN4,ENB); /* motor driver */

Ultrasonic ultrasonic(8);

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
	if (abs(Position_encoder - Inverted_pendulum_controller.GetPositionSetPoint()) <= 0)
	{
		Position_encoder = Inverted_pendulum_controller.GetPositionSetPoint();
	}

	if (abs(Angle_encoder - Inverted_pendulum_controller.GetAngleSetPoint()) <= 0)
	{
		Angle_encoder = Inverted_pendulum_controller.GetAngleSetPoint();
	}
	if (WITHIN_PID_SCOPE && Position_encoder >= 40 && Position_encoder <= 400)
	{
		output = Inverted_pendulum_controller.InvertedPendulumUpdate(Angle_encoder, Position_encoder);
		if (output > 0)
		{
			throttle = output + offset;
				if(throttle > 255) throttle = 255;
			DCMotor.backward(throttle,0);
		}else if (output < 0)
		{
			output = -1*output;
			throttle = output + offset;
				if(throttle > 255) throttle = 255;
			DCMotor.forward(throttle,0);
		}else
		{
			DCMotor.full_stop(0);
		}
	}else
	{
		DCMotor.full_stop(0);
	}
}
