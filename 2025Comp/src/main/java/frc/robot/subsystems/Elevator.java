// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */


  TalonSRX elevator1;
  TalonSRX elevator2;
  Boolean high = false;
  Boolean middle = false;
  Boolean low = false;
  Boolean stowed = true;
  DigitalInput highPosition;
  DigitalInput middlePosition;
  DigitalInput lowPosition;

  public Elevator() {

    elevator1 = new TalonSRX(Constants.ELEVATOR_IDS[0]);
    elevator2 = new TalonSRX(Constants.ELEVATOR_IDS[1]);

    elevator1.configPeakCurrentLimit(Constants.ELAVATOR_CURRENT_LIMIT);
    elevator2.configPeakCurrentLimit(Constants.ELAVATOR_CURRENT_LIMIT);
    elevator2.setInverted(true);

    elevator2.follow(elevator1);

    highPosition = new DigitalInput(Constants.ELEVATOR_POSITION[0]);
    middlePosition = new DigitalInput(Constants.ELEVATOR_POSITION[1]);
    lowPosition = new DigitalInput(Constants.ELEVATOR_POSITION[2]);

  }

  public void GoUp() {
    elevator1.set(TalonSRXControlMode.PercentOutput,Constants.ELEVATOR_MAX_UP_SPEED);
  }

  public void GoHigh() {}

  public void GoMiddle() {}

  public void GoLow() {}

  public void GoDown() {
    elevator1.set(TalonSRXControlMode.PercentOutput,Constants.ELEVATOR_MAX_DOWN_SPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
