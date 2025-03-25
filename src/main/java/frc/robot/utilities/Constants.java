// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);//14.5
  //public static final RobotConfig robotConfig;

  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class SpeedConstants
  {
    public static final double ELEVATOR_SPEED_UP = 0.75;
    public static final double CLAW_SPEED = 0.3;
    public static final double KICKER_SPEED = 0.80;
    public static final double FUNNEL_SPEED = 0.7;
    public static final double ACTUATOR_SPEED = 0.9;
    public static final double ELEVATOR_P_UP = 0.1;
    public static final double ELEVATOR_P_DOWN = 0.05;
    
  }

  public static class LimitConstants{
    public static final double ELEVATOR_HEIGHT_LIMIT = 83.75;
    public static final double ELEVATOR_CLAW_UP_HEIGHT_LIMIT = 31.88;
    public static final double CLAW_UPPER_LIMIT = 0.0;
    public static final double CLAW_LOWER_LIMIT = -0.5737;
    ;
  }

  public static class AbsoluteEncoderOffsets{
    public static final double ELEVATOR_OFFSET = 0.0;
    public static final double CLAW_OFFSET = 0.0;
    public static final double FUNNEL_OFFSET = 0.0;
  }

  public static class AutoConstants{
    public static final double HIGH_CORAL_CLAW_POSITION = -0.19061328125;
    public static final double MIDDLE_CORAL_CLAW_POSITION = -0.173359375;
    public static final double LOW_CORAL_CLAW_POSITION = -0.18;
    public static final double HOME_CORAL_CLAW_POSITION = 0.15;
    public static final double LOW_CORAL_ELEVATOR_POSITION = 30;
    public static final double HIGH_CORAL_ELEVATOR_POSITION = 84;
    public static final double MEDIUM_CORAL_ELEVATOR_POSITION = 55.89;
    public static final double HOME_ELEVATOR_POSITION = 1;

  }

  public static final class Auton {
    public static final String DEFAULT_AUTO_NAME = "MoveOut";

    public static final String[] AUTO_NAMES = {
      "MoveOut",
      "MoveAndPoint",
      "SmallTestAuto"
    };
  }

  
}
