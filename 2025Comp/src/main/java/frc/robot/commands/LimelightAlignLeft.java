// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.GyroSwerveDrive;

public class LimelightAlignLeft extends Command {
  private final GyroSwerveDrive m_gyroSwerveDrive;
  private boolean start = true;


  /** Creates a new LimelightAlign. */
  public LimelightAlignLeft(GyroSwerveDrive gyroSwerveDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    //driveStick = new XboxController(Constants.DRIVE_CONTROLLER_PORT);
    m_gyroSwerveDrive = gyroSwerveDrive;
    addRequirements(m_gyroSwerveDrive);
  }

  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
