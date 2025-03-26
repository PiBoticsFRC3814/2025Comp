// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class CoralAngle extends SubsystemBase {
  /** Creates a new CoralAngle. */

 public SparkMax angleMotor;
 private SparkClosedLoopController anglePIDController;
 public RelativeEncoder angleEncoder;
 public SparkLimitSwitch forwardAngleMotorLimit;
 public SparkLimitSwitch reverseAgnleMotorLimit;
 
 public SparkMaxConfig angleMotorConfig;
 public CoralAngle() {
    angleMotor = new SparkMax(Constants.CORAL_MOTOR_IDS[1], MotorType.kBrushless);
	  angleMotorConfig = new SparkMaxConfig();
		angleMotorConfig.voltageCompensation(Constants.SWERVE_VOLT_COMP);
		angleMotorConfig.idleMode(IdleMode.kBrake);
	  angleMotorConfig.inverted(false);
    angleMotorConfig.openLoopRampRate( 0.1 );
		angleMotorConfig.smartCurrentLimit(20, 30);
	
    angleMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    angleMotorConfig.closedLoop.pidf(Constants.CORAL_PID_CONSTANTS[0],
                                    Constants.CORAL_PID_CONSTANTS[1],
                                    Constants.CORAL_PID_CONSTANTS[2],
                                    Constants.CORAL_PID_CONSTANTS[4]);
    angleMotorConfig.closedLoop.iZone(Constants.CORAL_PID_CONSTANTS[3]);
    angleMotorConfig.closedLoop.outputRange(Constants.CORAL_PID_CONSTANTS[5],
                                    Constants.CORAL_PID_CONSTANTS[6]);
    angleMotorConfig.closedLoop.positionWrappingEnabled(true);
		angleMotorConfig.encoder.positionConversionFactor(Constants.CORAL_POSITION_FACTOR);
		angleMotorConfig.encoder.velocityConversionFactor(Constants.CORAL_VELOCITY_FACTOR);
		angleMotorConfig.encoder.uvwAverageDepth(4); // i think this is correct due to Neos using a hall-sensor encoder.
    angleMotorConfig.encoder.uvwMeasurementPeriod(16);

		angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		anglePIDController = angleMotor.getClosedLoopController();
		angleEncoder = angleMotor.getEncoder();
		angleEncoder.setPosition(0.0);

    forwardAngleMotorLimit = angleMotor.getForwardLimitSwitch();
    reverseAgnleMotorLimit = angleMotor.getReverseLimitSwitch();

 }

 public double getMotorAngleRadians(){
  return angleEncoder.getPosition();
 }

 public double getMotorAngleDegrees(){
  double degrees = (angleEncoder.getPosition() * (180/Math.PI));
  return degrees;
 }

 public void resetMotorAngle(){
  if(forwardAngleMotorLimit.isPressed()){
    angleEncoder.setPosition(0.0);
  }
 }

 public boolean goToMiddleCoral(){
  if (getMotorAngleDegrees() < Constants.CORAL_MIDDLE_ANGLE-0.1){
    angleMotor.set(setAngleMotorSpeed(Constants.CORAL_MIDDLE_ANGLE)+0.25);
    return false;
  }
  else if (getMotorAngleDegrees() > Constants.CORAL_MIDDLE_ANGLE+0.1){
    angleMotor.set(-setAngleMotorSpeed(Constants.CORAL_MIDDLE_ANGLE)-0.25); 
    return false; 
  }
  else {
    angleMotor.set(0.0);
    return true;
  }
 }

 public boolean goToTopCoral(){
  if (getMotorAngleDegrees() < Constants.CORAL_TOP_ANGLE-0.1){
    angleMotor.set(setAngleMotorSpeed(Constants.CORAL_TOP_ANGLE)+0.25);
    return false;
  }
  else if (getMotorAngleDegrees() > Constants.CORAL_TOP_ANGLE+0.1){
    angleMotor.set(-setAngleMotorSpeed(Constants.CORAL_TOP_ANGLE)-0.25);
    return false;  
  }
  else {
    angleMotor.set(0.0);
    return true;
  }
 }

 public boolean goToIntakeCoral(){
  if (getMotorAngleDegrees() < Constants.CORAL_INTAKE_ANGLE-0.1){
    angleMotor.set(setAngleMotorSpeed(Constants.CORAL_INTAKE_ANGLE)+0.25);
    return false;
  }
  else if (getMotorAngleDegrees() > Constants.CORAL_INTAKE_ANGLE+0.1){
    //experiment with -1 and +1 for range
    angleMotor.set(-setAngleMotorSpeed(Constants.CORAL_INTAKE_ANGLE)-0.25);  
    //it might not exactly reach the angle, so we need to add a range
    return false;
  }
  else {
    angleMotor.set(0.0);
    return true;
  }
 }

 public boolean goToArmedCoral(){
  return true;
 }

 public double setAngleMotorSpeed(double angle){
  double desiredAngle = angle;
  return 1.0*(Math.abs(getMotorAngleDegrees() - desiredAngle)/180);  //percent speed based on 180 degrees of range of motion
 }

 public void manualIncreaseAngle(){
  angleMotor.set(Constants.CORAL_ANGLE_UP_SPEED);
 }

 public void manualDecreaseAngle(){
  angleMotor.set(Constants.CORAL_ANGLE_DOWN_SPEED);
 }

 public void stopAngle(){
  angleMotor.set(0.0);
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
