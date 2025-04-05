// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TrapMotor extends SubsystemBase {
  /** Creates a new TrapMotor. */
  TalonSRX trapMotor;

  public TrapMotor() {
    trapMotor = new TalonSRX(Constants.CLIMB_MOTOR_IDS[2]);
    trapMotor.configPeakCurrentLimit(Constants.TRAP_MOTOR_CURRENT_LIMIT);
    trapMotor.setInverted(false);
    trapMotor.setNeutralMode(NeutralMode.Brake);
  }
  
public void TOpen() {
  trapMotor.set(TalonSRXControlMode.PercentOutput, Constants.TRAP_MAX_OPEN_SPEED);
}

public void TClose() {
  trapMotor.set(TalonSRXControlMode.PercentOutput, Constants.TRAP_MAX_CLOSE_SPEED);
}

public void TStop() {
  trapMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
