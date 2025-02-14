// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public SparkMax climberMotor1;
  public SparkMax climberMotor2;

  public SparkMaxConfig climberMotor1Config;
  public SparkMaxConfig climberMotor2Config;


  public Climber() {
    climberMotor1 = new SparkMax(Constants.CLIMB_MOTOR_IDS[0], MotorType.kBrushless);
    climberMotor2 = new SparkMax(Constants.CLIMB_MOTOR_IDS[1], MotorType.kBrushless);

    climberMotor1Config = new SparkMaxConfig();
    climberMotor1Config.idleMode(IdleMode.kBrake);
		climberMotor1Config.smartCurrentLimit(45, 35);

   
    climberMotor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    climberMotor1Config.closedLoop.pidf(Constants.CLIMB_PID_CONSTANTS[0],
                                        Constants.CLIMB_PID_CONSTANTS[1],
                                        Constants.CLIMB_PID_CONSTANTS[2],
                                        Constants.CLIMB_PID_CONSTANTS[4]);
    climberMotor1Config.closedLoop.iZone(Constants.CLIMB_PID_CONSTANTS[3]);
    climberMotor1Config.closedLoop.outputRange(Constants.CLIMB_PID_CONSTANTS[5],
                                        Constants.CLIMB_PID_CONSTANTS[6]);

    climberMotor1.configure(climberMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    climberMotor2Config.follow(climberMotor1,true);	

    climberMotor2.configure(climberMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void climbUp(){
    climberMotor1.set(0.1);
  }

  public void climbDown(){
    climberMotor1.set(-0.1);
  }

  public void climbStop(){
    climberMotor1.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
