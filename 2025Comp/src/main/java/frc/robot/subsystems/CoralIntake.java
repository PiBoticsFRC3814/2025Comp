// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
  /** Creates a new CoralIntake. */

  TalonSRX coralMotor;

  public CoralIntake() {
    coralMotor = new TalonSRX(Constants.CORAL_MOTOR_IDS[0]);

    coralMotor.configPeakCurrentLimit(Constants.CORAL_MOTOR_CURRENT_LIMIT);
    coralMotor.setInverted(false);
  }

  public void cIntake(){
    coralMotor.set(TalonSRXControlMode.PercentOutput, Constants.CORAL_MAX_IN_SPEED);
  }

  public void cOuttake(){
    coralMotor.set(TalonSRXControlMode.PercentOutput, Constants.CORAL_MAX_OUT_SPEED);
  }

  public void cStop(){
    coralMotor.set(TalonSRXControlMode.PercentOutput,0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
