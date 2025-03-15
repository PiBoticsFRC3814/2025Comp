// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public SparkMax climberMotor1;
  public SparkMax climberMotor2;
  public RelativeEncoder climberEncoder;
  private SparkClosedLoopController climbPIDController;

  public SparkMaxConfig climberMotor1Config;
  public SparkMaxConfig climberMotor2Config;


  public Climber() {
    climberMotor1 = new SparkMax(Constants.CLIMB_MOTOR_IDS[0], MotorType.kBrushless);
    climberMotor2 = new SparkMax(Constants.CLIMB_MOTOR_IDS[1], MotorType.kBrushless);

    climberMotor1Config = new SparkMaxConfig();
    climberMotor1Config.idleMode(IdleMode.kBrake);
		climberMotor1Config.smartCurrentLimit(80, 80);

   
    climberMotor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    climberMotor1Config.closedLoop.pidf(Constants.CLIMB_PID_CONSTANTS[0],
                                        Constants.CLIMB_PID_CONSTANTS[1],
                                        Constants.CLIMB_PID_CONSTANTS[2],
                                        Constants.CLIMB_PID_CONSTANTS[4]);
    climberMotor1Config.closedLoop.iZone(Constants.CLIMB_PID_CONSTANTS[3]);
    climberMotor1Config.closedLoop.outputRange(Constants.CLIMB_PID_CONSTANTS[5],
                                        Constants.CLIMB_PID_CONSTANTS[6]);

    climberMotor1Config.encoder.positionConversionFactor(Constants.CLIMB_POSITION_CONVERSION);
    climberMotor1Config.encoder.velocityConversionFactor(Constants.CLIMB_VELOCITY_CONVERSION);
		climberMotor1Config.encoder.uvwAverageDepth(4); // i think this is correct due to Neos using a hall-sensor encoder.
    climberMotor1Config.encoder.uvwMeasurementPeriod(16);

    climberMotor1.configure(climberMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    climbPIDController = climberMotor1.getClosedLoopController();

    climberEncoder = climberMotor1.getEncoder();
    climberEncoder.setPosition(0.0);

    climberMotor2Config = new SparkMaxConfig();
    climberMotor2Config.follow(climberMotor1,true);	

    climberMotor2.configure(climberMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public double getClimbDegrees(){
    double degrees = (climberEncoder.getPosition() * (180/Math.PI));
    return degrees;
   }

  public boolean climbUp(){
    if (getClimbDegrees() < Constants.CLIMB_MAX_ANGLE){
      climberMotor1.set(0.5);
      return false;
    }   else if (getClimbDegrees() >= Constants.CLIMB_MAX_ANGLE){ //Constants.CORAL_MIDDLE_ANGLE+1
      climberMotor1.set(0.0);
      System.out.println("made it1");
      return true;  
    }else{
      climberMotor1.set(0.0);
      System.out.println("made it2");
      return true;
    }
  }

  public void climbDown(){
    if (climberEncoder.getPosition() < Constants.CLIMB_MAX_ANGLE){
      climberMotor1.set(-0.4);
    } else{
      climberMotor1.set(0.0);
    }
   
  }

  public void climbStop(){
    climberMotor1.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
