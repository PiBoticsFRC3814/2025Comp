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
		{ 0.4, 0.01, 0.0, 0.1, 0.0, -1.0, 1.0 }, //FL
		{ 0.4, 0.01, 0.0, 0.1, 0.0, -1.0, 1.0 }, //FR
		{ 0.4, 0.01, 0.0, 0.1, 0.0, -1.0, 1.0 }, //RL
		{ 0.4, 0.01, 0.0, 0.1, 0.0, -1.0, 1.0 }  //RR
	};

  public static double[][] SWERVE_DRIVE_PID_CONSTANTS = { 
		// kP   kI   kD  kIz  kFF  kMn  kMx
		{ 0.4, 0.0, 0.2e-2, 0.0, 0.215, -1.0, 1.0 }, //FL //0.215
		{ 0.4, 0.0, 0.2e-2, 0.0, 0.215, -1.0, 1.0 }, //FR
		{ 0.4, 0.0, 0.2e-2, 0.0, 0.215, -1.0, 1.0 }, //RL
		{ 0.4, 0.0, 0.2e-2, 0.0, 0.215, -1.0, 1.0 }  //RR
	  //Should be FL, FR, RL, RR but should be able to have all modules with same tune - Alex
	};

  public static final double MAX_DRIVETRAIN_SPEED = 5820;

  public static final double[] TAG_ALIGN_STR_PID = {0.4, 0.0, 0.006};
  public static final double[] TAG_ALIGN_ROT_PID = {0.01, 5.0e-2, 0.002};
  public static final double[] TAG_ALIGN_FWD_PID = {0.4, 0.0, 0.006};

  public static final double AUTO_LEFT_OFFSET = 0.0;
  public static final double AUTO_RIGHT_OFFSET = 0.0;

  public static final boolean[] STEER_MOTOR_INVERTED = {false, false, false, false};
  public static final boolean[] DRIVE_MOTOR_INVERTED = {true, true, true, true};

  /*
   * Swerve constants for swerve module calculations
   */
  public static final double SWERVE_FRAME_WIDTH = 22;
  public static final double SWERVE_FRAME_LENGTH = 22;
  public static final double SWERVE_RADIUS = Math.sqrt(Math.pow(SWERVE_FRAME_LENGTH, 2) + Math.pow(SWERVE_FRAME_WIDTH, 2));
  public static final double SWERVE_PID_TOLERANCE = 2.8e-4;
  public static final double DRIVE_POSITION_CONVERSION = (0.0986 * Math.PI) / (6.75);
  public static final double DRIVE_VELOCITY_FACTOR = DRIVE_POSITION_CONVERSION / 60;
  public static final double STEER_POSITION_FACTOR = 2.0 * Math.PI / 12.8; 
  public static final double STEER_VELOCITY_FACTOR = STEER_POSITION_FACTOR / 60;
  public static final double MAX_SPEED_MperS = 4.56;
  public static final double MIN_SPEED_MperS = 0.05;
  public static final double MAX_TURN_SPEED_MperS = 1.0;
  public static final double MIN_TURN_SPEED_MperS = 0.05;
  public static final double Max_Anglular_Speed = 2.0; //change at some point

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
  public static final double[] ELEVATOR_PID_CONSTANTS = {1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0};
  public static final double ELEVATOR_POSITION_FACTOR = 1.196;
  public static final double ELEVATOR_VELOCITY_FACTOR = 1;
  public static final double ELEVATOR_MAX_UP_SPEED = 1.0;
  public static final double ELEVATOR_MAX_DOWN_SPEED = -1.0;
  public static final double ELEVATOR_HOME_SPEED = -0.3;

  //I swear these stupid values are gonna be the death of me save me java gods i beg -Gyrp
  public static final double ELEVATOR_PROCCESOR_DISTANCE = 0.0; //this is on the ground no elevator hight needed
  public static final double ELEVATOR_CORAL1_DISTANCE = 0.0; // ground
  public static final double ELEVATOR_CORAL2_DISTANCE = 85.0; // these are not staying the same the same... annoying.
  public static final double ELEVATOR_CORAL3_DISTANCE = 130.0; // 
  public static final double ELEVATOR_CORAL4_DISTANCE = 287.0; // 
  public static final double ELEVATOR_NET_DISTANCE = 0.0; // no ball so no net
  public static final double ELEVATOR_INTAKE_DISTANCE = 96; // 
  //FRAME is 11 inches away from tag

  public static final int[] ELEVATOR_POSITION = {0,1,2,3};

  public static final int[] CLIMB_MOTOR_IDS = {60,61,59};
  public static final double[] CLIMB_PID_CONSTANTS = {1.0e-1, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0};
  public static final double CLIMB_POSITION_CONVERSION = 2 * Math.PI / (5*5*4);
  public static final double CLIMB_VELOCITY_CONVERSION = 1.0;
  //public static final double CLIMB_MAX_ANGLE = 180.0; //175.0; //153.0;

  public static final int[] CORAL_MOTOR_IDS = {10,11,12,13};
  public static final int CORAL_MOTOR_CURRENT_LIMIT = 40;
  public static final int TRAP_MOTOR_CURRENT_LIMIT = 40;
  public static final double CORAL_MAX_IN_SPEED = 0.8;
  public static final double CORAL_MAX_OUT_SPEED = -0.8;
  public static final double TRAP_MAX_OPEN_SPEED = 1.0;
  public static final double TRAP_MAX_CLOSE_SPEED = -1.0;
  public static final double[] CORAL_PID_CONSTANTS = {1.0e-1, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0};
  public static final double CORAL_POSITION_FACTOR = 2.0 * Math.PI / (9*9*4); 
  public static final double CORAL_VELOCITY_FACTOR = CORAL_POSITION_FACTOR / (60);
  public static final double CORAL_MIDDLE_ANGLE = -1.533*180/Math.PI + 10;
  public static final double CORAL_TOP_ANGLE = -2.36*180/Math.PI + 10; //-2
  public static final double CORAL_INTAKE_ANGLE = -0.947*180/Math.PI +5;
  public static final double CORAL_ARMED_ANGLE = -0.0*180/Math.PI;
  public static final double CORAL_ANGLE_UP_SPEED = 0.2;
  public static final double CORAL_ANGLE_DOWN_SPEED = -0.2;
  

  public static final int ALGAE_MOTOR_CURRENT_LIMIT = 20;
  public static final double ALGAE_IN_SPEED = 0.5;
  public static final double ALGAE_OUT_SPEED = -0.5;
  public static final double[] ANGLE_PID_CONSTANTS = {1.0e-1, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0};

  public static final double AUTO_DRIVE_TIME = 2.0;  //time for auto to drive forward increase or decrease as needed
  public static final double AUTO_DRIVE_SPEED = 1.0; //speed in m/s that auto will drive froward increase or decrease as needed.

}

