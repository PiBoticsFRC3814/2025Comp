// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.GyroSwerveDrive;
import frc.robot.subsystems.RobotStates;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignLeft extends Command {
  /** Creates a new AutoAlignLeft. */
  boolean isValidTarget = false;
  boolean inProperPosition = false;
  double invertX = 1.0;
  double invertY = 1.0; 
  double xAdjust = 0.0;
  double yAdjust = 0.0;
  double lrAdjust = 0.0;
  double xDistance = 0.0;
  double yDistance = 0.0;
  double xSpeed = 0.0;
  double ySpeed = 0.0;
  double rotNeed = 0.0;
  double rotSpeed = 0.0;
  Rotation2d robotAngle;
  Pose2d robotLocation;
  Pose2d elementLocation;
  RobotStates m_robotStates;
  int tagID = 0;
  GyroSwerveDrive m_drive;
  double leftOffset = Constants.AUTO_LEFT_OFFSET;
  
  public AutoAlignLeft(GyroSwerveDrive drive, double adjust, RobotStates state) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    lrAdjust = adjust;
    m_robotStates = state;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    inProperPosition = false;
    if(m_drive.getAprilTagID()>0){
      tagID = m_drive.getAprilTagID();
      isValidTarget = true;
    } else {
      tagID = 0;
      isValidTarget = false;
      System.out.println("No Valid Target");
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isValidTarget){
      if (tagID == 8 || tagID == 9 || tagID == 20 || tagID == 19){
        xAdjust = lrAdjust*Math.cos(Math.toRadians(60));
      } else if(tagID == 6 || tagID == 11 || tagID == 22 || tagID == 17){
        xAdjust = -lrAdjust*Math.cos(Math.toRadians(60));
      } else if(tagID == 7 || tagID == 10 || tagID == 21 || tagID == 18){
        xAdjust = lrAdjust;
      } else{
        xAdjust = 0.0;
      }
      if (tagID == 8 || tagID == 6 || tagID == 20 || tagID == 22){
        yAdjust = lrAdjust*Math.sin(Math.toRadians(60));
      } else if(tagID == 9 || tagID == 11 || tagID == 19 || tagID == 17){
        yAdjust = -lrAdjust*Math.sin(Math.toRadians(60));
      } else if(tagID == 7 || tagID == 10 || tagID == 21 || tagID == 18){
        yAdjust = 0.0;
      } else{
        yAdjust = 0.0;
      }
      
      elementLocation = m_robotStates.fieldElementPose[tagID];
      robotLocation = m_drive.getPose();
    
      xDistance = robotLocation.getX() - (elementLocation.getX() + xAdjust);  //distance needed to trqavel in X
      yDistance = robotLocation.getY() - (elementLocation.getY() + yAdjust);  // distance needed to travel in Y

      //scaling x speed with minimum of 0.0.1 m/s and max speed of 4.56 m/s if 2 meters or more away
      xSpeed = Math.abs(xDistance*2.28)/2;
      if (xSpeed > Constants.MAX_SPEED_MperS/2){
        xSpeed = Constants.MAX_SPEED_MperS/2;
      }
      if (xSpeed < Constants.MIN_SPEED_MperS){
        xSpeed = Constants.MIN_SPEED_MperS;
      }
      //scaling y speed with minimum of 0.33 m/s and max speed of 4.56 m/s if 2 meters or more away
      ySpeed = Math.abs(yDistance*2.28); 
      if (ySpeed > Constants.MAX_SPEED_MperS/2) {
        ySpeed = Constants.MAX_SPEED_MperS/2;
      }
      if (ySpeed < Constants.MIN_SPEED_MperS){
        ySpeed = Constants.MIN_SPEED_MperS;
      }
    
      //if x or y distance is negative reverse direction
      if (xDistance < 0){
        xSpeed = -xSpeed;
      }
      if (yDistance < 0){
        ySpeed = -ySpeed;
      }

      //rotation speed max speed at 45 degrees?  i think?
      rotNeed = robotLocation.getRotation().getRadians() - elementLocation.getRotation().getRadians();
      rotSpeed = Math.abs(rotNeed*1.27)/2;
    
      if (rotSpeed > Constants.MAX_TURN_SPEED_MperS){
        rotSpeed = Constants.MAX_TURN_SPEED_MperS;
      }
      if (rotSpeed < Constants.MIN_TURN_SPEED_MperS){
        rotSpeed = Constants.MAX_TURN_SPEED_MperS;
      }

      if (rotNeed < 0.0){
        rotSpeed = -rotSpeed;
      }

      if(Math.abs(xDistance) < 0.01) {
        xSpeed = 0.0;
      }
      if(Math.abs(yDistance) < 0.01) {
        ySpeed = 0.0;
      }
      if(Math.abs(rotNeed) < 0.02) {
        rotSpeed = 0.0;
      }

      robotAngle = robotLocation.getRotation();

      m_drive.goToFieldElementLocation(xSpeed, ySpeed, rotSpeed, robotAngle);
      System.out.println("x " + xSpeed + " y " + ySpeed + " rot " + rotSpeed);
      if (xSpeed == 0.0 && ySpeed == 0.0 && rotSpeed == 0.0){
        System.out.println("Made It");
        inProperPosition = true;
      } else {
        System.out.println("Moving To Location");
      }
    } else {
      System.out.println("No Valid Target");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return inProperPosition;
  }
}
