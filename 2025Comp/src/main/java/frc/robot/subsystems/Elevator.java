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


  TalonSRX elevator1;
  TalonSRX elevator2;
  SparkMax elevator;
  SparkMaxConfig elevatorConfig;
  RelativeEncoder elevatorEncoder;
  SparkLimitSwitch forwardElevatorLimit;
  SparkLimitSwitch reverseElevatorLimit;
  SparkClosedLoopController elevatorPIDController;
  Boolean coral1 = false;
  Boolean coral2 = false;
  Boolean coral3 = false;
  Boolean coral4 = false;
  Boolean processor = false;
  Boolean net = false;
  Boolean stowed = true;
  Boolean intake = false;
  Boolean elevatorIsHomed = false;
  DigitalInput coral1Position;
  DigitalInput coral2Position;
  DigitalInput coral3Position;
  DigitalInput coral4Position;
  DigitalInput processorPosition;
  DigitalInput netPosition;
  DigitalInput intakePosition;
  double elevatorOffset = 0;
  public boolean elevatorAtPos = false;

  public Elevator() {
     elevator = new SparkMax( Constants.CORAL_MOTOR_IDS[1], MotorType.kBrushless );
	  elevatorConfig = new SparkMaxConfig();
		elevatorConfig.voltageCompensation(Constants.SWERVE_VOLT_COMP);
		elevatorConfig.idleMode(IdleMode.kBrake);
	  elevatorConfig.inverted(false);
		elevatorConfig.smartCurrentLimit(25, 25);
	
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



    //elevator1 = new TalonSRX(Constants.ELEVATOR_IDS[0]);
    //elevator2 = new TalonSRX(Constants.ELEVATOR_IDS[1]);

    //elevator1.configPeakCurrentLimit(Constants.ELAVATOR_CURRENT_LIMIT);
    //elevator2.configPeakCurrentLimit(Constants.ELAVATOR_CURRENT_LIMIT);
    //elevator2.setInverted(true);

    //elevator2.follow(elevator1);

    coral1Position = new DigitalInput(Constants.ELEVATOR_POSITION[0]);
    coral2Position = new DigitalInput(Constants.ELEVATOR_POSITION[1]);
    coral3Position = new DigitalInput(Constants.ELEVATOR_POSITION[2]);
    coral4Position = new DigitalInput(Constants.ELEVATOR_POSITION[3]);
    processorPosition = new DigitalInput(Constants.ELEVATOR_POSITION[4]);
    netPosition = new DigitalInput(Constants.ELEVATOR_POSITION[5]);
    intakePosition = new DigitalInput(Constants.ELEVATOR_POSITION[6]);
    //highPosition = new DigitalInput(Constants.ELEVATOR_POSITION[0]);
    //middlePosition = new DigitalInput(Constants.ELEVATOR_POSITION[1]);
    //lowPosition = new DigitalInput(Constants.ELEVATOR_POSITION[2]);

  }

  public void manualGoUp() {
    //elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_UP_SPEED);
    if (coral1Position.get()) {
      coral1 = true;
      coral2 = false;
      coral3 = false;
      coral4 = false;
      processor = false;
      net = false;
      stowed = false;
      intake = false;
    }
    if (coral2Position.get()) {
      coral1 = false;
      coral2 = true;
      coral3 = false;
      coral4 = false;
      processor = false;
      net = false;
      stowed = false;
      intake = false;
    }
    if (coral3Position.get()) {
      coral1 = false;
      coral2 = false;
      coral3 = true;
      coral4 = false;
      processor = false;
      processor = false;
      net = false;
      stowed = false;
      intake = false;
    }
    if (coral4Position.get()) {
      coral1 = false;
      coral2 = false;
      coral3 = false;
      coral4 = true;
      processor = false;
      net = false;
      stowed = false;
      intake = false;
    }
    if (processorPosition.get()) {
      coral1 = false;
      coral2 = false;
      coral3 = false;
      coral4 = false;
      processor = true;
      net = false;
      stowed = false;
      intake = false;
    }
    if (netPosition.get()) {
      coral1 = false;
      coral2 = false;
      coral3 = false;
      coral4 = false;
      processor = false;
      net = true;
      stowed = false;
      intake = false;
    }
    if (intakePosition.get()) {
      coral1 = false;
      coral2 = false;
      coral3 = false;
      coral4 = false;
      processor = false;
      net = false;
      stowed = false;
      intake = true;
    }
  
  }

  public void manualGoDown() {
    //elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_DOWN_SPEED);
    if (coral1Position.get()) {
      coral1 = true;
      coral2 = false;
      coral3 = false;
      coral4 = false;
      processor = false;
      net = false;
      stowed = false;
      intake = false;
    }
    if (coral2Position.get()) {
    coral1 = false;
    coral2 = true;
    coral3 = false;
    coral4 = false;
    processor = false;
    net = false;
    stowed = false;
    intake = false;
    }
    if (coral3Position.get()) {
      coral1 = false;
      coral2 = false;
      coral3 = true;
      coral4 = false;
      processor = false;
      processor = false;
      net = false;
      stowed = false;
      intake = false;
    }
    if (coral4Position.get()) {
      coral1 = false;
      coral2 = false;
      coral3 = false;
      coral4 = true;
      processor = false;
      net = false;
      stowed = false;
      intake = false;
    }
    if (processorPosition.get()) {
      coral1 = false;
      coral2 = false;
      coral3 = false;
      coral4 = false;
      processor = true;
      net = false;
      stowed = false;
      intake = false;
    }
    if (netPosition.get()) {
      coral1 = false;
      coral2 = false;
      coral3 = false;
      coral4 = false;
      processor = false;
      net = true;
      stowed = false;
      intake = false;
    }
    if (intakePosition.get()) {
      coral1 = false;
      coral2 = false;
      coral3 = false;
      coral4 = false;
      processor = false;
      net = false;
      stowed = false;
      intake = true;
    }
  }

  public void stopElevator(){
    //elevator1.set(TalonSRXControlMode.PercentOutput, 0.0);
  } 

  public int getLastKnownPosistion(){
    if (coral1) {
      return 2;
    }
    else if (coral2) {
      return 3;
    }
    else if (coral3) {
      return 5;
    }
    else if (coral4) {
      return 6;
    }
    else if (processor){
      return 1;
    }
    else if (net){
      return 7;
    }
    else if (stowed){
      return 0;
    }
    else if (intake){
      return 4;
    }
    else return 0;
  
  }

  public boolean goToCoral1 (boolean direction) {
    if (direction) {
      if (!coral1Position.get()){
        //elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_UP_SPEED);
        return coral1;
      }
      else {
        //elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = true;
        coral2 = false;
        coral3 = false;
        coral4 = false;
        processor = false;
        net = false;
        stowed = false;
        intake = false;
        return coral1;
      }
    }
    else {
      if (!coral1Position.get()){
        //elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_DOWN_SPEED);
        return coral1;
      }
      else {
       //elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = true;
        coral2 = false;
        coral3 = false;
        coral4 = false;
        processor = false;
        net = false;
        stowed = false;
        intake = false;
        return coral1;
      }
    }
  }

  public boolean goToCoral2 (boolean direction) {
    if (direction) {
      if (!coral2Position.get()){
        //elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_UP_SPEED);
        return coral2;
      }
      else {
        //elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = true;
        coral3 = false;
        coral4 = false;
        processor = false;
        net = false;
        stowed = false;
        intake = false;
        return coral2;
      }
    }
    else {
      if (!coral2Position.get()){
        //elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_DOWN_SPEED);
        return coral2;
      }
      else {
        //elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = true;
        coral3 = false;
        coral4 = false;
        processor = false;
        net = false;
        stowed = false;
        intake = false;
        return coral2;
      }
    }
  }

  public boolean goToCoral3 (boolean direction) {
    if (direction) {
      if (!coral3Position.get()){
        //elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_UP_SPEED);
        return coral3;
      }
      else {
       //elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = false;
        coral3 = true;
        coral4 = false;
        processor = false;
        net = false;
        stowed = false;
        intake = false;
        return coral3;
      }
    }
    else {
      if (!coral3Position.get()){
        //elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_DOWN_SPEED);
        return coral3;
      }
      else {
        //elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = false;
        coral3 = true;
        coral4 = false;
        processor = false;
        net = false;
        stowed = false;
        intake = false;
        return coral3;
      }
    }
  }

  public boolean goToCoral4 (boolean direction) {
    if (direction) {
      if (!coral4Position.get()){
        //elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_UP_SPEED);
        return coral4;
      }
      else {
        //elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = false;
        coral3 = false;
        coral4 = true;
        processor = false;
        net = false;
        stowed = false;
        intake = false;
        return coral4;
      }
    }
    else {
      if (!coral4Position.get()){
        //elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_DOWN_SPEED);
        return coral4;
      }
      else {
        //elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = false;
        coral3 = false;
        coral4 = true;
        processor = false;
        net = false;
        stowed = false;
        intake = false;
        return coral4;
      }
    }
  }
  
  public boolean goToProccesor (boolean direction) {
    if (direction) {
      if (!processorPosition.get()){
        //elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_UP_SPEED);
        return processor;
      }
      else {
        //elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = false;
        coral3 = false;
        coral4 = false;
        processor = true;
        net = false;
        stowed = false;
        intake = false;
        return processor;
      }
    }
    else {
      if (!processorPosition.get()){
        //elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_DOWN_SPEED);
        return processor;
      }
      else {
        //elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = false;
        coral3 = false;
        coral4 = false;
        processor = true;
        net = false;
        stowed = false;
        intake = false;
        return processor;
      }
    }
  }

  public boolean goToNet (boolean direction) {
    if (direction) {
      if (!netPosition.get()){
        //elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_UP_SPEED);
        return net;
      }
      else {
        //elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = false;
        coral3 = false;
        coral4 = false;
        processor = false;
        net = true;
        stowed = false;
        intake = false;
        return net;
      }
    }
    else {
      if (!netPosition.get()){
        //elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_DOWN_SPEED);
        return net;
      }
      else {
        //elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = false;
        coral3 = false;
        coral4 = false;
        processor = false;
        net = true;
        stowed = false;
        intake = false;
        return net;
      }
    }
  }

  public boolean GoToIntake (boolean direction) {
    if (direction) {
      if (!intakePosition.get()){
        //elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_UP_SPEED);
        return intake;
      }
      else {
       //elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = false;
        coral3 = false;
        coral4 = false;
        processor = false;
        net = false;
        stowed = false;
        intake = true;
        return intake;
      }
    }
    else {
      if (!intakePosition.get()){
        //elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_DOWN_SPEED);
        return intake;
      }
      else {
        //elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = false;
        coral3 = false;
        coral4 = false;
        processor = false;
        net = false;
        stowed = false;
        intake = true;
        return intake;
      }
    }
  }

   public boolean ArmDistance(double position) {
    if(!elevatorIsHomed){
      elevator.set(-Constants.ELEVATOR_HOME_SPEED);
      if(!reverseElevatorLimit.isPressed()){
        elevator.set(0.0);
        elevatorOffset = elevatorEncoder.getPosition();
        elevatorIsHomed = true;
        DriverStation.reportError("Is homed", false);
      }
    } else{
      elevatorPIDController.setReference(position + elevatorOffset, ControlType.kPosition);
    }
    System.out.println("position " + elevatorEncoder.getPosition() + " offset " + elevatorOffset + " desired " + position);
    return Math.abs(elevatorEncoder.getPosition() - elevatorOffset - position) <=10;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
