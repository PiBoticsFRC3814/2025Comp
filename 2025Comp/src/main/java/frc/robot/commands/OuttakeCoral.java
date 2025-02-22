// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OuttakeCoral extends Command {
  private final CoralIntake m_Outtake;

  public OuttakeCoral(CoralIntake subsystem) {
    m_Outtake = subsystem;
    addRequirements(m_Outtake);
  }

  @Override
  public void initialize() {
    // Initialize the intake motor if needed
  }

  @Override
  public void execute() {
    // Run the intake motor
    m_Outtake.cOuttake();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the intake motor when the command ends
    m_Outtake.cStop();
  }

  @Override
  public boolean isFinished() {
    // This command never finishes on its own
    return false;
  }
}
