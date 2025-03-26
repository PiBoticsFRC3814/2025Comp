// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


public class SwerveModule {
	public  SparkMax            driveMotor;
  	public  SparkMaxConfig      driveMotorConfig;
	private SparkClosedLoopController driveVelocityPIDController;  //i this needed anymore?  seems the closed loop control is handeled in the config
	private SparkClosedLoopController steerPIDController;

	public  SparkMax            steerMotor;
  	public  SparkMaxConfig      steerMotorConfig;
	private CANcoder              steerAngleEncoder;
	private SimpleMotorFeedforward driveFF;

	public double                 position;
	private int 				  index;
	public RelativeEncoder 		  driveEncoder;
	public RelativeEncoder 		  steerEncoder;

	public static double   STATUS_TIMEOUT_SECONDS = 0.02;
	
	/* the SwerveModule subsystem */
	public SwerveModule( int swerveModIndex ) {
		driveMotor = new SparkMax( Constants.SWERVE_DRIVE_MOTOR_IDS[swerveModIndex], MotorType.kBrushless);
    	driveMotorConfig = new SparkMaxConfig();
    	driveMotorConfig.inverted(Constants.DRIVE_MOTOR_INVERTED[swerveModIndex]);
    	driveMotorConfig.idleMode(IdleMode.kBrake);
		//driveMotor.IdleMode(IdleMode.kBrake);
    	//driveMotor.setIdleMode(IdleMode.kBrake);
    	driveMotorConfig.voltageCompensation(Constants.SWERVE_VOLT_COMP);
		//driveMotor.enableVoltageCompensation(Constants.SWERVE_VOLT_COMP);
		//driveMotor.setInverted( Constants.DRIVE_MOTOR_INVERTED[swerveModIndex] );
		driveMotorConfig.openLoopRampRate( 0.1 );
		driveMotorConfig.smartCurrentLimit(45, 35);

    //Stupid old Rev Config System. Will need to update because they broke it in 2025 - Alex
		//driveVelocityPIDController = driveMotor.getClosedLoopController();
		//driveVelocityPIDController.setP(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][0]);
		//driveVelocityPIDController.setI(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][1]);
		//driveVelocityPIDController.setD(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][2]);
		//driveVelocityPIDController.setIZone(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][3]); 
		//driveVelocityPIDController.setFF(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][4]);
		//driveVelocityPIDController.setOutputRange(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][5], Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][6]);

    //new REV PID stuff?
    	driveMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    	driveMotorConfig.closedLoop.pidf(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][0],Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][1],Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][2],Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][4]);
    	driveMotorConfig.closedLoop.iZone(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][3]);
    	driveMotorConfig.closedLoop.outputRange(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][5],Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][6]);


		//driveEncoder = driveMotor.getEncoder();
		driveMotorConfig.encoder.positionConversionFactor(Constants.DRIVE_POSITION_CONVERSION);
		driveMotorConfig.encoder.velocityConversionFactor(Constants.DRIVE_VELOCITY_FACTOR);
		driveMotorConfig.encoder.uvwAverageDepth(4); // i think this is correct due to Neos using a hall-sensor encoder.
    	driveMotorConfig.encoder.uvwMeasurementPeriod(16);
		configureCANStatusFrames(10, 20, 20, 500, 500, 200, 200, driveMotorConfig); //Magic numbers other teams and YAGSL uses. Makes things happier
		//driveMotor.burnFlash();
		
		driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		driveVelocityPIDController = driveMotor.getClosedLoopController();
		driveEncoder = driveMotor.getEncoder();

		steerMotor = new SparkMax( Constants.SWERVE_STEER_MOTOR_IDS[swerveModIndex], MotorType.kBrushless );
		steerMotorConfig = new SparkMaxConfig();
		steerMotorConfig.voltageCompensation(Constants.SWERVE_VOLT_COMP);
		steerMotorConfig.idleMode(IdleMode.kBrake);
		steerMotorConfig.inverted( Constants.STEER_MOTOR_INVERTED[swerveModIndex] );
		steerMotorConfig.smartCurrentLimit(25, 25);

		//steerPIDController.setP(Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][0]);
		//steerPIDController.setI(Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][1]);
		//steerPIDController.setD(Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][2]);
		//steerPIDController.setIZone(Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][3]); 
		//steerPIDController.setOutputRange(Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][5], Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][6]);
		//steerPIDController.setPositionPIDWrappingEnabled(true);
		//steerPIDController.setPositionPIDWrappingMinInput(0);
		//steerPIDController.setPositionPIDWrappingMaxInput(2 * Math.PI);
		steerMotorConfig.closedLoop.pidf(Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][0],Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][1],Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][2],Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][4]);
		steerMotorConfig.closedLoop.iZone(Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][3]);
		steerMotorConfig.closedLoop.outputRange(Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][5],Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][6]);
		steerMotorConfig.closedLoop.positionWrappingEnabled(true);
		
		//steerEncoder = steerMotor.getEncoder();
		//steerEncoder.setPositionConversionFactor(Constants.STEER_POSITION_FACTOR);
		//steerEncoder.setVelocityConversionFactor(Constants.STEER_VELOCITY_FACTOR);
		//steerEncoder.setAverageDepth(4);
		//steerEncoder.setMeasurementPeriod(16);
		//configureCANStatusFrames(10, 20, 20, 500, 500, 200, 200, steerMotor);
		//steerMotor.burnFlash();
		steerMotorConfig.encoder.positionConversionFactor(Constants.STEER_POSITION_FACTOR);
		steerMotorConfig.encoder.velocityConversionFactor(Constants.STEER_VELOCITY_FACTOR);
		steerMotorConfig.encoder.uvwAverageDepth(4); // i think this is correct due to Neos using a hall-sensor encoder.
    	steerMotorConfig.encoder.uvwMeasurementPeriod(16);

		steerAngleEncoder = new CANcoder( Constants.SWERVE_ENCODER_IDS[swerveModIndex] );

		steerMotor.configure(steerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		steerPIDController = steerMotor.getClosedLoopController();
		steerEncoder = steerMotor.getEncoder();
		steerEncoder.setPosition(getAbsolutePosition());

		index = swerveModIndex;
	}
	public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(getStateAngle()));
    }

	public void configureCANStatusFrames(
      int CANStatus0, int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4, int CANStatus5, int CANStatus6, SparkMaxConfig config)
  {
    config.signals.appliedOutputPeriodMs(CANStatus0);
	config.signals.motorTemperaturePeriodMs(CANStatus1);
	config.signals.primaryEncoderPositionPeriodMs(CANStatus2);
	config.signals.analogVoltagePeriodMs(CANStatus3);
	config.signals.externalOrAltEncoderPosition(CANStatus4);
	config.signals.absoluteEncoderPositionPeriodMs(CANStatus5);
	config.signals.absoluteEncoderVelocityPeriodMs(CANStatus6);
    //  https://docs.revrobotics.com/brushless/spark-max/control-interfaces
  }


	public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(getStateAngle()));
    }

	public void output(){
		SmartDashboard.putNumber("speed mod " + index, driveEncoder.getVelocity());
		SmartDashboard.putNumber("angle mod " + index, new Rotation2d(getStateAngle()).getDegrees());
		SmartDashboard.putNumber("absolute" + index, (getAbsolutePosition()*180/3.14));
	}

	public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state;
		state = desiredState;
		state.optimize(new Rotation2d(getStateAngle()));
	if((Math.abs(state.angle.getRadians() - getStateAngle()) < Math.toRadians(1.0) ) && steerEncoder.getVelocity() < Math.toRadians(5.0)){
		steerMotor.set(0);
	} else {
		setReferenceAngle(state.angle.getRadians());
	}
	if(Math.abs(state.speedMetersPerSecond) < 0.05){
		driveMotor.set(0);
	} else {
		double velocity = RobotState.isAutonomous() ? getCosineCompensatedVelocity(state) : state.speedMetersPerSecond;
		//Applies compensation to prevent unintended sideways motion and reduce slippage when module is not at setpoint position
		//Does not run during teleop due to it making direction changes far more aggressive and driver will naturally compensate for slight skewing
		//and drift
        driveVelocityPIDController.setReference(velocity, ControlType.kVelocity);
        	//driveVelocityPIDController.setReference(Constants.MAX_SPEED_MperS, ControlType.kVelocity);
	}
    	}
	
	private double getCosineCompensatedVelocity(SwerveModuleState desiredState){
    double cosineScalar = 1.0;
    // Taken from the CTRE SwerveModule class.
    // https://api.ctr-electronics.com/phoenix6/release/java/src-html/com/ctre/phoenix6/mechanisms/swerve/SwerveModule.html#line.46
    /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
    /* To reduce the "skew" that occurs when changing direction */
    /* If error is close to 0 rotations, we're already there, so apply full power */
    /* If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help us at all */
    cosineScalar = Rotation2d.fromDegrees(desiredState.angle.getDegrees())
                             .minus(new Rotation2d(getStateAngle()))
                             .getCos(); // TODO: Investigate angle modulus by 180.
    /* Make sure we don't invert our drive, even though we shouldn't ever target over 90 degrees anyway */
    if (cosineScalar < 0.0)
    {
      cosineScalar = 1;
    }

    return desiredState.speedMetersPerSecond * (cosineScalar);
  }

	public void resetModule(){
		steerEncoder.setPosition(getAbsolutePosition());
		setDesiredState(new SwerveModuleState(0.0, new Rotation2d(0)));
	}

	public void setReferenceAngle(double referenceAngleRadians) {
        double currentAngleRadians = steerEncoder.getPosition();

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Neo's encoder can go above
        // that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }
        steerPIDController.setReference(adjustedReferenceAngleRadians, ControlType.kPosition);
        //steerPIDController.setReference(0.0, ControlType.kPosition);
    }


	public double getStateAngle() {
        double motorAngleRadians = steerEncoder.getPosition();
        motorAngleRadians %= 2.0 * Math.PI;
        if (motorAngleRadians < 0.0) {
            motorAngleRadians += 2.0 * Math.PI;
        }

        return motorAngleRadians;
    }

public double getAbsolutePosition()
  {

    StatusSignal<Angle> angle = steerAngleEncoder.getAbsolutePosition();

    // Taken from democat's library.
    // Source: https://github.com/democat3457/swerve-lib/blob/7c03126b8c22f23a501b2c2742f9d173a5bcbc40/src/main/java/com/swervedrivespecialties/swervelib/ctre/CanCoderFactoryBuilder.java#L51-L74
    for (int i = 0; i < 10; i++)
    {
      if (angle.getStatus() == StatusCode.OK)
      {
        break;
      }
      angle = angle.waitForUpdate(STATUS_TIMEOUT_SECONDS);
    }

    return angle.getValueAsDouble() * Math.PI * 2.0;
  }

  public void initDefaultCommand() {
    // NOTE: no default command unless running swerve modules seperately
  }
}
