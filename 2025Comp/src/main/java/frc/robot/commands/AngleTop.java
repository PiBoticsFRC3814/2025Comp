// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralAngle;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AngleTop extends Command {
  /** Creates a new AngleTop. */
  CoralAngle m_Angle;
  double angleDegrees = 0;
  boolean finished = false;
  public AngleTop(CoralAngle angle) {
    m_Angle = angle;
    addRequirements(angle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleDegrees = m_Angle.getMotorAngleDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finished = m_Angle.goToTopCoral();
    SmartDashboard.putBoolean("made it", finished);  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Angle.stopAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
