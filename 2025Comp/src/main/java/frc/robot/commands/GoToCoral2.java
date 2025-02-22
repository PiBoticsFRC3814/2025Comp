// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToCoral2 extends Command {
  /** Creates a new GoToCoral2. */
  int position = 0;
  Elevator m_elevator;
  boolean inPosition = false;
  boolean direction = false;
  
  public GoToCoral2(Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;  
    addRequirements(elevator);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    inPosition = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    inPosition = m_elevator.ArmDistance(Constants.ELEVATOR_CORAL2_DISTANCE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (inPosition){
      m_elevator.stopElevator();
    }
    return inPosition;
  }
}
