// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import  frc.robot.Constants;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

public class SwerveModuleSimulationConfig extends SubsystemBase {
  public final DCMotor driveMotor;
  public final DCMotor steerMotor;
  public final double driveGearRatio;
  public final double steerGearRatio;
  public final double driveFrictionVoltage;
  public final double steerFrictionVoltage;
  public final double wheelRadius;
  public final double steerMOI;
  public final double wheelCOF;

  /** Creates a new SwerveModuleSimulationConfig. */
  public SwerveModuleSimulationConfig(DCMotor driveMotor, DCMotor steerMotor, double driveGearRatio, double steerGearRatio,
                                      double driveFrictionVoltage, double steerFrictionVoltage, double wheelRadius,
                                      double steerMOI, double wheelCOF) {
    this.driveMotor = DCMotor.getNEO(1);
    this.steerMotor = DCMotor.getNEO(1);
    this.driveGearRatio = driveGearRatio; // get the rest at some point
    this.steerGearRatio = steerGearRatio;
    this.driveFrictionVoltage = driveFrictionVoltage;
    this.steerFrictionVoltage = steerFrictionVoltage;
    this.wheelRadius = wheelRadius;
    this.steerMOI = steerMOI; // VERY difficult to calculate but is important. If RB not there ask DL for help
    this.wheelCOF = wheelCOF; // same here :shrug:
  }
 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
