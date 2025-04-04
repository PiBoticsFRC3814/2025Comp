// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.GyroSwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoMoveOffLine extends Command {
  /** Creates a new MoveOffLine. */
  boolean start = false;
  boolean done = false;
  GyroSwerveDrive m_drive;
  Timer time;
  double invert = 0.0;

  
  public AutoMoveOffLine(GyroSwerveDrive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    time = new Timer();
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //this will invert the X speed depending on the side of the field we are on speed will be based on the contantas AUTO_DRIVE_SPEED variable
    invert = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? -Constants.AUTO_DRIVE_SPEED : Constants.AUTO_DRIVE_SPEED;

    //timer initialization and start
    start = true;
    done = false;
    time.reset();
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //move forward for AUTO_DRIVE_TIME at AUTO_DRIVE_SPEED
    if (time.get() < Constants.AUTO_DRIVE_TIME){
      m_drive.drive(invert,0.0,0.0,false,false,0.0);
      done = false;
    } else {
      m_drive.drive(0.0,0.0,0.0,false,false,0.0);
      done = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0.0,0.0,m_drive.getGyroAngle(),false,false,0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    start = false;
    return done;
  }
}
