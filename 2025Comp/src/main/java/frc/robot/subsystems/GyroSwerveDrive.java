// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import java.io.ObjectInputFilter.Config;

import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.config.*;
import com.pathplanner.lib.util.*;
import com.revrobotics.spark.config.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class GyroSwerveDrive extends SubsystemBase {
  private double[] speed = {0.0, 0.0, 0.0, 0.0};
  private double[] angle = {0.0, 0.0, 0.0, 0.0};
  private RobotStates m_RobotStates;
  
  private SlewRateLimiter slewX = new SlewRateLimiter(Constants.JOYSTICK_X_SLEW_RATE);
  private SlewRateLimiter slewY = new SlewRateLimiter(Constants.JOYSTICK_Y_SLEW_RATE);

  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator poseEstimator;
  private ADIS16470_IMU gyro;
  PIDController turnController = new PIDController(0.02, 0.1, 0.001);

  RobotConfig config;

  public boolean trustVision;
  public Pose2d currentPose;
  double avgTrust;

  private SwerveModule[] swerveMod = {
    new SwerveModule(0), new SwerveModule(1), new SwerveModule(2), new SwerveModule(3)
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

    //AutoBuilder.configureHolonomic(
    //  this::getPose,
    //  this::resetOdometry,
    //  this::getChassisSpeed,
    //  this::setModuleStates(),
    //  new HolonomicPathFollowerConfig(
    //    new PIDConstants(9, 0.0, 0.3),
    //    new PIDConstants(11.0, 0.0, 0.1),
    //    Constants.MAX_SPEED_MperS,
    //    Constants.SWERVE_RADIUS / 25.4 / 1000.0,
    //    new ReplanningConfig()
    //  ),
    //  () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    // Mote this needs to be looked at per game since it can be morrored or rotated red vs blue.  2025 is rotated not mirrored.

     //               var alliance = DriverStation.getAlliance();
     //               if (alliance.isPresent()) {
     //                   return alliance.get() == DriverStation.Alliance.Red;
     //               }
     //               return false;
     //           },
     //           this
    //);
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

    poseEstimator.updateWithTime(
      Timer.getFPGATimestamp(),
       Rotation2d.fromDegrees(gyro.getAngle(gyro.getYawAxis()) % 360.0),
        getModulePositions()
    );

    double ampdistance = (Math.sqrt(Math.pow(Math.abs(getPose().getX()) - 1.65,2.0) + Math.pow(getPose().getY() - 7.54,2.0)) * 1000.0 / 25.4);
    m_RobotStates.speakDist = ampdistance;
    //values from linear regression given datapoints causes I'm too lazy
    //0 1750
    //3 1800
    //6 1900
    //12 2000
    ampdistance = ampdistance >= 0.0 ? ampdistance : 0.0;
    m_RobotStates.inAmp = ampdistance <= 12;
    m_RobotStates.ampSpeed = (ampdistance >= 1.0 ? 740.16 * Math.log(186.236 * ampdistance + 5334.16) - 4607.85 : 1750.0);
    double Speakerdistance;
    try{
      Speakerdistance = 325 - Math.sqrt(Math.pow(Math.abs(getPose().getX() - 8.308975),2.0) + Math.pow(getPose().getY() - (!m_RobotStates.autonomous && DriverStation.getAlliance().get() == DriverStation.Alliance.Red  ? -1.442593 : 1.442593),2.0)) * 1000 / 25.4;
    } catch(Exception e){
      Speakerdistance = 0;
      System.out.println(e);
    }
    //values from linear regression given datapoints causes I'm too lazy
    //28 3650 -0.1
    //10 3900 -0.1
    //0 4500 0.0
    Speakerdistance = Speakerdistance >= 0.0 ? Speakerdistance : 0.0;
    m_RobotStates.inSpeaker = Speakerdistance <= 24;
    m_RobotStates.speakSpeed = Speakerdistance >= 3 ? 1.1 * (-259.36 * Math.log(0.00721146 * (Speakerdistance) + 0.00791717) + 3245.03) : 4600;
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
    //m_FLModule.setDesiredState(desiredStates[0]);
    //m_FRModule.setDesiredState(desiredStates[1]);
    //m_RLModule.setDesiredState(desiredStates[2]);
    //m_RRModule.setDesiredState(desiredStates[3]);
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

  public void drive(double xSpeed, double ySpeed, double setAngle, boolean lock, boolean speakerLock, double zSpeed) {
    //xSpeed = slewX.calculate(xSpeed);
    //ySpeed = slewY.calculate(ySpeed);

    Pose2d position = getPose();
    if(speakerLock){setAngle = Math.toDegrees(Math.atan2(position.getY() - 5.45, position.getY()));}
    double rot = 0.0;
    if(lock || speakerLock) rot = -turnController.calculate(setAngle, position.getRotation().getDegrees());
    rot *= Constants.MAX_SPEED_MperS / new Rotation2d(Constants.SWERVE_FRAME_LENGTH / 2.0 * 0.0254, Constants.SWERVE_FRAME_WIDTH / 2.0 * 0.0254).getRadians();
    setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, position.getRotation()));
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
}
