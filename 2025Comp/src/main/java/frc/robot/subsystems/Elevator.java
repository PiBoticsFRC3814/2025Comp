// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */


  SparkMax elevator;
  SparkMaxConfig elevatorConfig;
  public RelativeEncoder elevatorEncoder;
  SparkLimitSwitch forwardElevatorLimit;
  SparkLimitSwitch reverseElevatorLimit;
  SparkClosedLoopController elevatorPIDController;
  boolean elevatorIsHomed = true;
  boolean elevatorAtPos = false;
  DigitalInput elevatorDownLimit;
  double elevatorOffset = 0;

  public Elevator() {
    elevator = new SparkMax( Constants.ELEVATOR_IDS[0], MotorType.kBrushless );
	  elevatorConfig = new SparkMaxConfig();
		elevatorConfig.voltageCompensation(Constants.SWERVE_VOLT_COMP);
		elevatorConfig.idleMode(IdleMode.kBrake);
	  elevatorConfig.inverted(false);
		elevatorConfig.smartCurrentLimit(60, 60);
	
    elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    elevatorConfig.closedLoop.pidf(Constants.ELEVATOR_PID_CONSTANTS[0],
                                    Constants.ELEVATOR_PID_CONSTANTS[1],
                                    Constants.ELEVATOR_PID_CONSTANTS[2],
                                    Constants.ELEVATOR_PID_CONSTANTS[4]);
    elevatorConfig.closedLoop.iZone(Constants.ELEVATOR_PID_CONSTANTS[3]);
    elevatorConfig.closedLoop.outputRange(Constants.ELEVATOR_PID_CONSTANTS[5],
                                    Constants.ELEVATOR_PID_CONSTANTS[6]);
    elevatorConfig.closedLoop.positionWrappingEnabled(true);
		elevatorConfig.encoder.positionConversionFactor(Constants.ELEVATOR_POSITION_FACTOR);
		elevatorConfig.encoder.velocityConversionFactor(Constants.ELEVATOR_VELOCITY_FACTOR);
		elevatorConfig.encoder.uvwAverageDepth(4); // i think this is correct due to Neos using a hall-sensor encoder.
    elevatorConfig.encoder.uvwMeasurementPeriod(16);

		elevator.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		elevatorPIDController = elevator.getClosedLoopController();
		elevatorEncoder = elevator.getEncoder();
		elevatorEncoder.setPosition(0.0);

    forwardElevatorLimit = elevator.getForwardLimitSwitch();
    reverseElevatorLimit = elevator.getReverseLimitSwitch();
    elevatorDownLimit = new DigitalInput(Constants.ELEVATOR_POSITION[0]);

  }

  public void manualGoUp() {
    elevator.set(Constants.ELEVATOR_MAX_UP_SPEED);   
  }

  public void manualGoDown() {
    elevator.set(Constants.ELEVATOR_MAX_DOWN_SPEED);   
  }

  public void stopElevator(){
    elevator.set(0.0);
  } 

  public void homeElevator(){
    if (!elevatorDownLimit.get()){
      elevator.set(Constants.ELEVATOR_MAX_DOWN_SPEED);
    } else {
      elevator.set(0.0);
      elevatorEncoder.setPosition(0.0);
      elevatorIsHomed = true;
    }
  }

  public boolean ArmDistance(double position) {
    elevatorIsHomed = false;
    //elevatorPIDController.setReference(position, ControlType.kPosition);
    //System.out.println("position " + elevatorEncoder.getPosition() + " offset " + elevatorOffset + " desired " + position);
    //return Math.abs(elevatorEncoder.getPosition() - position) <= 10;
    if (position - elevatorEncoder.getPosition() >= 1.0){
      elevator.set(1.0);
      return false;
    } else{
      elevator.set(0.1);
      return true;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
