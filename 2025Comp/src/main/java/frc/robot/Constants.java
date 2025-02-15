// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  ////////////////////////////////////////
  //                 OI                 //
  ////////////////////////////////////////

  public static final double JOYSTICK_X_DEADZONE = 0.15;
  public static final double JOYSTICK_Y_DEADZONE = 0.15;
  public static final double JOYSTICK_Z_DEADZONE = 0.15;
  public static final double JOYSTICK_Z2_DEADZONE = 0.15;
  public static final double JOYSTICK_X_SLEW_RATE = 100.0;
  public static final double JOYSTICK_Y_SLEW_RATE = 100.0;

  public static final int DRIVE_CONTROLLER_PORT = 2;
  public static final int STEER_CONTROLLER_PORT = 0;
  public static final double SLOW_SPEED = 0.5;
  public static final double FAST_SPEED = 1.0;


  ////////////////////////////////////////
  //               Swerve               //
  ////////////////////////////////////////

  /*
   * Swerve module motor and encoder ids
   * {FL,FR,RL,RR}
   */
  public static final int[] SWERVE_DRIVE_MOTOR_IDS = {23, 21, 20, 22};//{22, 23, 20, 21}; {21, 22, 20, 23}
  public static final int[] SWERVE_STEER_MOTOR_IDS = {33, 31, 30, 32};//{32, 33, 30, 31}; {31, 32, 30, 33};
  public static final int[] SWERVE_ENCODER_IDS =     {43, 41, 40, 42};//{42, 43, 40, 41}; {41, 42, 40, 43};
  

  public static final int swerveModuleNumber = 4;

  public static final double[] SWERVE_SETPOINT_OFFSET = {
    // must be between 0 & 360 degrees
	  //Do not use. Set offset in Phoenix Tuner X by orienting modules then hit zero - Alex
    0,//88.7, // Front Right
    0,//96.5, // Rear Right
    0,//360.0 - 126.2, // Rear Left
    0,//360.0 - 58.0 // Front Left
  };

  public static final double[][] SWERVE_STEER_PID_CONSTANTS = {
    // kP   kI   kD  kIz  kFF  kMn  kMx
		{ 1.0, 0.2e-1, 0.0, 0.1, 0.0, -1.0, 1.0 }, //Front Right
		{ 1.0, 0.2e-1, 0.0, 0.1, 0.0, -1.0, 1.0 }, //Rear Right
		{ 1.0, 0.2e-1, 0.0, 0.1, 0.0, -1.0, 1.0 }, //Rear Left
		{ 1.0, 0.2e-1, 0.0, 0.1, 0.0, -1.0, 1.0 }  //Front Left
	};

  public static double[][] SWERVE_DRIVE_PID_CONSTANTS = { 
		// kP   kI   kD  kIz  kFF  kMn  kMx
		{ 0.4, 0.0, 0.2e-2, 0.0, 0.215, -1.0, 1.0 }, //Front Right //0.215
		{ 0.4, 0.0, 0.2e-2, 0.0, 0.215, -1.0, 1.0 }, //Rear Right
		{ 0.4, 0.0, 0.2e-2, 0.0, 0.215, -1.0, 1.0 }, //Rear Left
		{ 0.4, 0.0, 0.2e-2, 0.0, 0.215, -1.0, 1.0 }  //Front Left
	  //Should be FL, FR, RL, RR but should be able to have all modules with same tune - Alex
	};

  public static final double MAX_DRIVETRAIN_SPEED = 5820;

  public static final double[] TAG_ALIGN_STR_PID = {0.4, 0.0, 0.006};
  public static final double[] TAG_ALIGN_ROT_PID = {0.01, 5.0e-2, 0.002};
  public static final double[] TAG_ALIGN_FWD_PID = {0.4, 0.0, 0.006};

  public static final boolean[] STEER_MOTOR_INVERTED = {false, false, false, false};
  public static final boolean[] DRIVE_MOTOR_INVERTED = {false, true, false, false};

  /*
   * Swerve constants for swerve module calculations
   */
  public static final double SWERVE_FRAME_WIDTH = 20.5;
  public static final double SWERVE_FRAME_LENGTH = 25.5;
  public static final double SWERVE_RADIUS = Math.sqrt(Math.pow(SWERVE_FRAME_LENGTH, 2) + Math.pow(SWERVE_FRAME_WIDTH, 2));
  public static final double SWERVE_PID_TOLERANCE = 2.8e-4;
  public static final double DRIVE_POSITION_CONVERSION = (0.0986 * Math.PI) / (6.75);
  public static final double DRIVE_VELOCITY_FACTOR = DRIVE_POSITION_CONVERSION / 60;
  public static final double STEER_POSITION_FACTOR = 2.0 * Math.PI / 12.8; 
  public static final double STEER_VELOCITY_FACTOR = STEER_POSITION_FACTOR / 60;
  public static final double MAX_SPEED_MperS = 4.17;

  public static final double[] DRIVE_FF = {
  // kS  kP   kV
    0.0, 0.0, 1.0 / (MAX_DRIVETRAIN_SPEED * DRIVE_VELOCITY_FACTOR)
  };
  //                                      (wheel circum / (encppr * swerve Ratio)

  public static final double SWERVE_VOLT_COMP = 12.6;

  // public static double[][] SHOOT_PID = { 
		// kP   kI   kD  kIz  kFF  kMn  kMx
	//	{ 1.0e-4, 0.0, 2.0e-4, 0.0, 1.65e-4, -1.0, 1.0 }, //Front Right
	//	{ 1.0e-4, 0.0, 2.0e-4, 0.0, 1.65e-4, -1.0, 1.0 }, //Rear Right
	//	{ 1.0e-4, 0.0, 2.0e-4, 0.0, 1.65e-4, -1.0, 1.0 }, //Rear Left
	//	{ 1.0e-4, 0.0, 2.0e-4, 0.0, 1.65e-4, -1.0, 1.0 }  //Front Left
	//};
  public static double kRotTransFactor = 0.045; 


  public static final int[] ELEVATOR_IDS = {50,51};
  public static final int ELAVATOR_CURRENT_LIMIT = 50;
  public static final double ELEVATOR_MAX_UP_SPEED = 1.0;
  public static final double ELEVATOR_MAX_DOWN_SPEED = -1.0;


  public static final int[] ELEVATOR_POSITION = {0,1,2,3,4,5};

  public static final int[] CLIMB_MOTOR_IDS = {60,61};
  public static final double[] CLIMB_PID_CONSTANTS = {1.0e-1, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0};

  public static final int[] CORAL_MOTOR_IDS = {70,71,72,73};
  public static final int CORAL_MOTOR_CURRENT_LIMIT = 20;
  public static final double CORAL_MAX_IN_SPEED = 0.5;
  public static final double CORAL_MAX_OUT_SPEED = -0.5;
  public static final int ALGAE_MOTOR_CURRENT_LIMIT = 20;
  public static final double ALGAE_IN_SPEED = 0.5;
  public static final double ALGAE_OUT_SPEED = -0.5;
  public static final double[] ANGLE_PID_CONSTANTS = {1.0e-1, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0};
  public static final double ALGAE_POSITION_FACTOR = 2.0 * Math.PI / (9*9*4); 
  public static final double ALGAE_VELOCITY_FACTOR = ALGAE_POSITION_FACTOR / (60);
  public static final double ALGEA_MIDDLE_ANGLE = -45.0;
  public static final double ALGEA_TOP_ANGLE = -90.0;
  public static final double ALGEA_INTAKE_ANGLE = -40.0;
  
}

