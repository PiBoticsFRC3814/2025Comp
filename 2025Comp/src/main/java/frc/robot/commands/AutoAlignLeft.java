// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.GyroSwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignLeft extends Command {
  /** Creates a new AutoAlignLeft. */
  boolean isValidTarget = false;
  boolean inProperPosition = false;
  GyroSwerveDrive m_drive;
  double leftOffset = Constants.AUTO_LEFT_OFFSET;
  
  public AutoAlignLeft(GyroSwerveDrive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    inProperPosition = false;
    if(m_drive.getAprilTagID()>0){
      isValidTarget = true;
    } else {
      isValidTarget = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isValidTarget){
      inProperPosition = m_drive.goToFieldElementLocation(Constants.AUTO_LEFT_OFFSET);
      System.out.println("moving to location");
    } else {
      System.out.println("no Valid Target");
      inProperPosition = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return inProperPosition;
  }
}
