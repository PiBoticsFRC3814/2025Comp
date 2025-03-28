// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
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
  DigitalInput[] elevatorLimit;
  int elevatorIndex = 0;

  double elevatorOffset = 0;

  public Elevator() {
    elevator = new SparkMax( Constants.ELEVATOR_IDS[0], MotorType.kBrushless );
	  elevatorConfig = new SparkMaxConfig();
		elevatorConfig.voltageCompensation(Constants.SWERVE_VOLT_COMP);
		elevatorConfig.idleMode(IdleMode.kBrake);
	  elevatorConfig.inverted(false);
		elevatorConfig.smartCurrentLimit(40, 40);
	
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
    elevatorLimit = new DigitalInput[4];
    elevatorLimit[0] = new DigitalInput(Constants.ELEVATOR_POSITION[0]);
    elevatorLimit[1] = new DigitalInput(Constants.ELEVATOR_POSITION[1]);
    elevatorLimit[2] = new DigitalInput(Constants.ELEVATOR_POSITION[2]);
    elevatorLimit[3] = new DigitalInput(Constants.ELEVATOR_POSITION[3]);

  }

  public void manualGoUp() {
    elevator.set(Constants.ELEVATOR_MAX_UP_SPEED);   
  }

  public void manualGoDown() {
    System.out.println("down");
    if (elevatorEncoder.getPosition() > 0.0){
      elevator.set(Constants.ELEVATOR_MAX_DOWN_SPEED);
    } else{
    elevator.set(0.0);
    }   
  }

  public void stopElevator(){
    elevator.set(0.0);
  } 

  public boolean ArmDistance(double position) {
    elevatorIsHomed = false;
    //elevatorPIDController.setReference(position, ControlType.kPosition);
    //System.out.println("position " + elevatorEncoder.getPosition() + " offset " + elevatorOffset + " desired " + position);
    //return Math.abs(elevatorEncoder.getPosition() - position) <= 10;
    if (position - elevatorEncoder.getPosition() >= 0.2){
      elevator.set(1.0);
      System.out.println(elevatorEncoder.getPosition());
      return false;
    } else{
      elevator.set(0.0);
      System.out.println("elevator at hight");
      return true;
    }
  }

  public boolean LimitDistance(int index){
    elevatorIndex = index;
    if(elevatorLimit[index].get()){
      elevator.set(1.0);
      return false;
    } else {
      elevator.set(0.0);
      return true;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
