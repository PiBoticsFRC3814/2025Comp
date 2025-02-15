// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */

  TalonSRX algaeMotor1;
  TalonSRX algaeMotor2;

  public AlgaeIntake() {
    algaeMotor1 = new TalonSRX(Constants.CORAL_MOTOR_IDS[2]);
    algaeMotor1 = new TalonSRX(Constants.CORAL_MOTOR_IDS[3]);

    algaeMotor1.configPeakCurrentLimit(Constants.ALGAE_MOTOR_CURRENT_LIMIT);
    algaeMotor2.configPeakCurrentLimit(Constants.ALGAE_MOTOR_CURRENT_LIMIT);
    algaeMotor1.setInverted(true);
    algaeMotor2.setInverted(false);

    algaeMotor2.follow(algaeMotor1);
  }

  public void aIntake(){
    algaeMotor1.set(TalonSRXControlMode.PercentOutput, Constants.ALGAE_IN_SPEED);
  }

  public void aOutake(){
    algaeMotor1.set(TalonSRXControlMode.PercentOutput, Constants.ALGAE_OUT_SPEED);
  }

  public void algaeStop(){
    algaeMotor1.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
