// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class RobotStates extends SubsystemBase {
  /** Creates a new RobotStates. */
  public boolean inFrontOfCubeStation;
  public int moveFromLastAlign;
  public boolean autonomous;
  public double driveMultiplier;
  public int gyroReset;
  public boolean center;
  public Pose2d[] fieldElementPose;
  public static final double leftOffset = 0.0;
  public static final double rightOffset = 0.33;


  public RobotStates() {
    inFrontOfCubeStation = false;
    moveFromLastAlign = 0;
    autonomous = false;
    driveMultiplier = Constants.SLOW_SPEED;
    gyroReset = 0;
    center = true;
    fieldElementPose = new Pose2d[23];
    fieldElementPose[0] = new Pose2d(0.0,0.0, Rotation2d.fromDegrees(0)); // dont use this pose array indexed at 0 but april tag indexed at 1 so this is an empty pose
    fieldElementPose[1] = new Pose2d(16.7,0.7, Rotation2d.fromDegrees(0)); // this is the pose for red left choral intake (perspective of red drivers)
    fieldElementPose[2] = new Pose2d(16.7,7.4, Rotation2d.fromDegrees(0)); // this is the pose for red right choral intake
    fieldElementPose[3] = new Pose2d(11.6,8.1, Rotation2d.fromDegrees(0)); // this is the pose for red proccessor
    fieldElementPose[4] = new Pose2d(9.3,6.1, Rotation2d.fromDegrees(0)); // this is the pose for red side blue barge
    fieldElementPose[5] = new Pose2d(9.3,1.9, Rotation2d.fromDegrees(0)); // this is the pose for red side red barge
    fieldElementPose[6] = new Pose2d(13.5,3.3, Rotation2d.fromDegrees(0)); // red side front left reef (from perspective of drivers)
    fieldElementPose[7] = new Pose2d(13.9,3.3, Rotation2d.fromDegrees(0)); // red side front reef
    fieldElementPose[8] = new Pose2d(13.5,4.7, Rotation2d.fromDegrees(0)); // red side front right reef
    fieldElementPose[9] = new Pose2d(12.6,4.7, Rotation2d.fromDegrees(0)); // red side back right reef
    fieldElementPose[10] = new Pose2d(12.2,4.0, Rotation2d.fromDegrees(0)); // red side back reef
    fieldElementPose[11] = new Pose2d(12.6,3.3, Rotation2d.fromDegrees(0)); // red side back left reef
    fieldElementPose[12] = new Pose2d(0.9,0.7, Rotation2d.fromDegrees(0)); // blue side right choral intake (perspective of blue drivers)
    fieldElementPose[13] = new Pose2d(0.9,7.4, Rotation2d.fromDegrees(0)); // blue side left choral intake
    fieldElementPose[14] = new Pose2d(8.3,6.1, Rotation2d.fromDegrees(0)); // blue side blue barge
    fieldElementPose[15] = new Pose2d(8.3,1.9, Rotation2d.fromDegrees(0)); // blue side red barge
    fieldElementPose[16] = new Pose2d(6.0,0.0, Rotation2d.fromDegrees(0)); // blue side proccessor
    fieldElementPose[17] = new Pose2d(4.1,3.3, Rotation2d.fromDegrees(0)); // blue front right reef
    fieldElementPose[18] = new Pose2d(3.7,4.0, Rotation2d.fromDegrees(0)); // blue front reef
    fieldElementPose[19] = new Pose2d(4.1,4.7, Rotation2d.fromDegrees(0)); // blue front left reef
    fieldElementPose[20] = new Pose2d(4.9,4.7, Rotation2d.fromDegrees(0)); // blue back left reef
    fieldElementPose[21] = new Pose2d(5.3,4.0, Rotation2d.fromDegrees(0)); // blue back reef
    fieldElementPose[22] = new Pose2d(5.3,3.3, Rotation2d.fromDegrees(0)); // blue back right reef
  

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}