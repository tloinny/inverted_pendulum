#include <AS5600.h>
#include <easy_inverted_pendulum.h>
#include <L298N.h>
#include <Encoder.h>
#include <Wire.h>

#define SWING_UP_RANG 140
#define WITHIN_PID_SCOPE Angle_encoder > Inverted_pendulum_controller.GetAngleSetPoint() - SWING_UP_RANG / 2 && Angle_encoder < Inverted_pendulum_controller.GetAngleSetPoint() + SWING_UP_RANG / 2
#define WITHIN_SWING_SCOPE Angle_encoder <= Inverted_pendulum_controller.GetAngleSetPoint() - SWING_UP_RANG / 2 || Angle_encoder >= Inverted_pendulum_controller.GetAngleSetPoint() + SWING_UP_RANG / 2

const int ENA = 7;
const int IN1 = 8;
const int IN2 = 9;
const int IN3 = 0;
const int IN4 = 0;
const int ENB = 0;	/* only one motor */

L298N DCMotor(ENA,IN1,IN2,IN3,IN4,ENB);	/* motor driver */

Encoder Encoder(5, 6);	/* position encoder */
AS5600 ams_5600;	/* angle encoder */
inverted_pendulum Inverted_pendulum_controller(0,0,0,0,255,255);	/* inverted pendulum pid controller */

float Last_angle = 0;
float Angle_encoder = 0;
float Position_encoder = 0;

float output = 0;

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
	ams_5600.setStartPosition(word(0/0.087));
	ams_5600.setEndPosition(word(355/0.087));	/* set the range of as5600 */
	Inverted_pendulum_controller.SetAngleSetPoint(0);
	Inverted_pendulum_controller.SetPositionSetPoint(0);

	/*
	 *Swing up
	 */
	do
	{
		Last_angle = Angle_encoder;
		Angle_encoder = convertScaledAngleToDegrees(ams_5600.getScaledAngle());
	}
	while(WITHIN_SWING_SCOPE)
	{
		if(Last_angle > Angle_encoder)
		{
			DCMotor.forward(255,100);
		}else
		{
			DCMotor.backward(255,100);
		}
		Last_angle = Angle_encoder;
		Angle_encoder = convertScaledAngleToDegrees(ams_5600.getScaledAngle());	
	}
	Encoder.write(Inverted_pendulum_controller.GetPositionSetPoint());
}

void loop()
{
	Angle_encoder = convertScaledAngleToDegrees(ams_5600.getScaledAngle());
	Position_encoder = Encoder.read();
	if(WITHIN_PID_SCOPE)
	{
		output = InvertedPendulumUpdate(Angle_encoder, Position_encoder);
		if (output > 0)
		{
			DCMotor.forward(output,0);
		}else if (output < 0)
		{
			DCMotor.backward(output,0);
		}else
		{
			DCMotor.full_stop(0);
		}
	}
}
