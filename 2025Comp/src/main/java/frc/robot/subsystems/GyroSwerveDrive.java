// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import com.pathplanner.lib.config.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
//import edu.wpi.first.math.kinematics.ChassisSpeeds

public class GyroSwerveDrive extends SubsystemBase {
  /*private double[] speed = {0.0, 0.0, 0.0, 0.0};
  private double[] angle = {0.0, 0.0, 0.0, 0.0};*/
  private RobotStates m_RobotStates;
  
  private SlewRateLimiter slewX = new SlewRateLimiter(Constants.JOYSTICK_X_SLEW_RATE);
  private SlewRateLimiter slewY = new SlewRateLimiter(Constants.JOYSTICK_Y_SLEW_RATE);

  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator poseEstimator;
  private ADIS16470_IMU gyro;
  PIDController turnController = new PIDController(0.04, 0.05, 0.010);

  RobotConfig config;

  public boolean trustVision;
  public Pose2d currentPose;
  double avgTrust;

  public PIDController translationXController;
  public PIDController translationYController;
  public ProfiledPIDController translationRotController;
  public double xAdjust = 0.0;
  public double yAdjust = 0.0;
  public boolean xSpeed;
  public Pose2d fieldElementLocation;
  public double ChassisSpeed;

  private SwerveModule[] swerveMod = {
    new SwerveModule(0), 
    new SwerveModule(1), 
    new SwerveModule(2), 
    new SwerveModule(3)
  }; // this builds the swerve module in entirety (both drive motor and steer motor) from the SwerveModule.java -- builds 4 of them based on normmal number of swerve modules

  public GyroSwerveDrive(RobotStates robotStates, ADIS16470_IMU gyro) {
    m_RobotStates = robotStates;
    this.gyro = gyro;
    
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    fieldElementLocation = m_RobotStates.fieldElementPose[0];
    translationXController = new PIDController(Constants.TAG_ALIGN_FWD_PID[0], 
                                                Constants.TAG_ALIGN_STR_PID[1],
                                                Constants.TAG_ALIGN_ROT_PID[2]);
    translationYController = new PIDController(Constants.TAG_ALIGN_FWD_PID[0], 
                                                Constants.TAG_ALIGN_STR_PID[1],
                                                Constants.TAG_ALIGN_ROT_PID[2]);
    translationRotController = new ProfiledPIDController(Constants.TAG_ALIGN_FWD_PID[0], 
                                                          Constants.TAG_ALIGN_STR_PID[1],
                                                          Constants.TAG_ALIGN_ROT_PID[2],
                                                          new Constraints(1.0, 0.1));

    turnController.setIntegratorRange(-0.2, 0.2);
    turnController.enableContinuousInput(0.0, 360.0);
    turnController.setTolerance(Math.toRadians(1.0));

    kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.SWERVE_FRAME_LENGTH / 2.0 * 0.0254, Constants.SWERVE_FRAME_WIDTH / 2.0 * 0.0254),     //FL
       new Translation2d(Constants.SWERVE_FRAME_LENGTH / 2.0 * 0.0254, -Constants.SWERVE_FRAME_WIDTH / 2.0 * 0.0254),   //FR
        new Translation2d(-Constants.SWERVE_FRAME_LENGTH / 2.0 * 0.0254, Constants.SWERVE_FRAME_WIDTH / 2.0 * 0.0254),  //RL
         new Translation2d(-Constants.SWERVE_FRAME_LENGTH / 2.0 * 0.0254, -Constants.SWERVE_FRAME_WIDTH / 2.0 * 0.0254) //RR
    ); //physical locations of the mnodules -- +x towards front, +y towards left -- appears to be in millimeters 0.0245 divider -- this should be usable to tell us if our mmodules are called out correctly.
       //From Alex: this is correct. WPiLib is FL, FR, RL, RR
    poseEstimator = new SwerveDrivePoseEstimator(
      kinematics, 
      Rotation2d.fromDegrees(gyro.getAngle(gyro.getYawAxis())),
       getModulePositions(),
        new Pose2d(),
          VecBuilder.fill(0.1, 0.1, 0.05),
            VecBuilder.fill(0.5, 0.5, 1.0)); //Alex: This needs to be properly setup. Look up Extended Kalman Filter for tuning

    trustVision = false;
  

    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::setModuleStates,//(speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(9.0, 0.0, 0.3), // Translation PID constants
                    new PIDConstants(11.0, 0.0, 0.1) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
            
            
    );
    
  }

  //help

  public ChassisSpeeds getChassisSpeed() {
    return kinematics.toChassisSpeeds(
      swerveMod[0].getState(),  //FL
      swerveMod[1].getState(),  //FR
      swerveMod[2].getState(),  //RL
      swerveMod[3].getState()   //RR
    );
  }
// cut my life into pieces this is my last resort!! :p -bleh
  @Override
  public void periodic() {
    if(LimelightHelpers.getTV("limelight")){
      LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      try{
        avgTrust = 0;
        for(int i = 0; i<limelightMeasurement.tagCount; i++) avgTrust += limelightMeasurement.rawFiducials[i].ambiguity;
        avgTrust/=limelightMeasurement.tagCount;
        if(limelightMeasurement.avgTagDist <= 3)updateVisionPoseEstimator(limelightMeasurement.pose, limelightMeasurement.timestampSeconds, avgTrust);
      } catch(Exception e){
        System.err.println(e);
      }
    }
    
    swerveMod[0].output();
    swerveMod[1].output();
    swerveMod[2].output();
    swerveMod[3].output();

    poseEstimator.updateWithTime(
      Timer.getFPGATimestamp(),
       Rotation2d.fromDegrees(gyro.getAngle(gyro.getYawAxis()) % 360.0),
        getModulePositions()
    );

  }

  public Pose2d getPose(){
    return poseEstimator.getEstimatedPosition();
  }

  private SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = swerveMod[0].getPosition();  //FL
    positions[1] = swerveMod[1].getPosition();  //FR
    positions[2] = swerveMod[2].getPosition();  //RL
    positions[3] = swerveMod[3].getPosition();  //RR
    return positions;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_SPEED_MperS);
    swerveMod[0].setDesiredState(desiredStates[0]);  //FL
    swerveMod[1].setDesiredState(desiredStates[1]);  //FR
    swerveMod[2].setDesiredState(desiredStates[2]);  //RL
    swerveMod[3].setDesiredState(desiredStates[3]);  //RR
  }

  public void resetModules(){
    swerveMod[0].resetModule();  //FL
    swerveMod[1].resetModule();  //FR
    swerveMod[2].resetModule();  //RL
    swerveMod[3].resetModule();  //RR
  }
  
  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    //*
    SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(secondOrderKinematics(chassisSpeeds));
    if(RobotState.isAutonomous()) desiredStates = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(chassisSpeeds, 0.02));
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_SPEED_MperS);
    swerveMod[0].setDesiredState(desiredStates[0]);  //FL
    swerveMod[1].setDesiredState(desiredStates[1]);  //FR
    swerveMod[2].setDesiredState(desiredStates[2]);  //RL
    swerveMod[3].setDesiredState(desiredStates[3]);  //RR
  }

  public ChassisSpeeds secondOrderKinematics(ChassisSpeeds chassisSpeeds) {
    Translation2d translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    Translation2d rotAdj = translation.rotateBy(new Rotation2d(-Math.PI / 2.0))
        .times(chassisSpeeds.omegaRadiansPerSecond * Constants.kRotTransFactor);

    translation = translation.plus(rotAdj);

    return new ChassisSpeeds(translation.getX(), translation.getY(), chassisSpeeds.omegaRadiansPerSecond);
  }


  public void resetOdometry(Pose2d pose){
    poseEstimator.resetPosition(
      Rotation2d.fromDegrees(gyro.getAngle(gyro.getYawAxis()) % 360.0),
       getModulePositions(),
        pose
    );
  }

  public void drive(double xSpeed, double ySpeed, double setAngle, boolean lock, double zSpeed) {
    //xSpeed = slewX.calculate(xSpeed);
    //ySpeed = slewY.calculate(ySpeed);

    Pose2d position = getPose();
    //position.getRotation();
    //if(speakerLock){setAngle = Math.toDegrees(Math.atan2(position.getY() - 5.45, position.getY()));}
    double rot = 0.0;
    if(lock) rot = -turnController.calculate(setAngle, position.getRotation().getDegrees());
    rot *= Constants.MAX_TURN_SPEED_MperS / new Rotation2d(Constants.SWERVE_FRAME_LENGTH / 2.0 * 0.0254, Constants.SWERVE_FRAME_WIDTH / 2.0 * 0.0254).getRadians();
    if(m_RobotStates.autonomous){
      rot = zSpeed;
    }    
    setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, position.getRotation())); // third value use rot for stick angle use zSpeed for manual turn
    //Rotation2d.fromDegrees(getGyroAngle()) instead of pose.getRotation() seems to yield same results.  there seems to be nothing to hold the angle consistant.

  }

  public void resetGyro(){
    gyro.reset();
    poseEstimator.resetPosition(Rotation2d.fromDegrees(0), getModulePositions(), getPose());
    m_RobotStates.gyroReset++;
  }

  public void updateVisionPoseEstimator(Pose2d visionEstimate, double timestamp, double trust){
    //ramp measurement trust based on robot distance
    //poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.1 * Math.pow(15, distance), 0.1 * Math.pow(15, distance), Units.degreesToRadians(20)));
    //if(tagNumber == 1)poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.8,.8, Math.toRadians(20)));
    //if(tagNumber > 1)poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.1,.1,Math.toRadians(5)));
    //0.04 0.04 5
    //0.8 0.8 20
    if(trust <= 0.8 && 1.0 >= Math.sqrt(getChassisSpeed().vxMetersPerSecond * getChassisSpeed().vxMetersPerSecond + getChassisSpeed().vyMetersPerSecond * getChassisSpeed().vyMetersPerSecond)){
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.1 + 5 * (trust <= 0.3 ? 0.0 : trust) ,.1 + 5 * (trust <= 0.3 ? 0.0 : trust),Math.toRadians(5) + 20 * trust));
      poseEstimator.addVisionMeasurement(visionEstimate, timestamp);
    }
  }

  private double applyDeadzone(double input, double deadzone) {
    if (Math.abs(input) < deadzone) return 0.0;
    double result = (Math.abs(input) - deadzone) / (1.0 - deadzone);
    return (input < 0.0 ? -result : result);
  }

  public void motorZero(){
    for (int i = 0; i < 4; i++) {
      swerveMod[i].driveMotor.set(0.0);
      swerveMod[i].steerMotor.set(0.0);
    }
  }

  public int getAprilTagID(){
    double id = LimelightHelpers.getFiducialID("limelight");
    return (int)id;
  }
  
  public Pose2d getFieldElementLocation(){
    int id = getAprilTagID();
    if (id > 0){
      fieldElementLocation = m_RobotStates.fieldElementPose[id];
      return fieldElementLocation;
    } else{
      return fieldElementLocation;
    }
  }

  public void goToFieldElementLocation(double xSpeed, double ySpeed, double rotSpeed, Rotation2d angle) { 
    //this will try to go to a specific april tag and/or scoring location.  this is going to need ALOT of dbugging.
    double xApplied = xSpeed;
    double yApplied = ySpeed;
    double rotApplied = rotSpeed;
    Rotation2d robotAngle = angle;

    setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xApplied, yApplied, rotApplied, robotAngle));
    
  }
  
  public double getGyroAngle(){
    return gyro.getAngle(gyro.getYawAxis()) % 360;
  } 
 
}
