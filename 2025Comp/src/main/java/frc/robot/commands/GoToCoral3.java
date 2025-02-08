// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToCoral3 extends Command {
  /** Creates a new GoToCoral3. */
  int position = 0;
  Elevator m_elevator;
  boolean inPosition = false;
  boolean direction = false;

  public GoToCoral3(Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;  
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position = m_elevator.getLastKnownPosistion();
    direction = false;
    inPosition = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    position = m_elevator.getLastKnownPosistion();
    if (position <4) {
      direction = true;
      m_elevator.goToCoral1(direction);
      inPosition = false;
    }
    else if (position >4) {
      direction = false;
      m_elevator.goToCoral1(direction);
      inPosition = false;
    }
    else {
      m_elevator.stopElevator();
      inPosition = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_elevator.stopElevator();
    return inPosition;
  }
}
