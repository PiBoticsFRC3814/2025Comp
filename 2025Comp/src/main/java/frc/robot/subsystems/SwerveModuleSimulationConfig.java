// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import frc.robot.Constants;



public class SwerveModuleSimulationConfig implements Supplier<SwerveModuleSimulation> {

    public final SimMotorConfigs driveMotorConfigs, steerMotorConfigs;

    public final double DRIVE_GEAR_RATIO, STEER_GEAR_RATIO, WHEELS_COEFFICIENT_OF_FRICTION;
    public final Voltage DRIVE_FRICTION_VOLTAGE;
    public final Distance WHEEL_RADIUS;
    
    public static void check(double value, double min, double max, String parameterName, String unitDescription) {

        if (value < min || value > max) {

            throw new IllegalArgumentException(

                String.format(

                    "%s (%f %s) is out of bounds. Must be between %f and %f %s.",

                    parameterName, value, unitDescription, min, max, unitDescription

                )

            );

        }

    }



    public SwerveModuleSimulationConfig(

            DCMotor driveMotorModel,

            DCMotor steerMotorModel,

            double driveGearRatio,

            double steerGearRatio,

            Voltage driveFrictionVoltage,

            Voltage steerFrictionVoltage,

            Distance wheelRadius,

            MomentOfInertia steerRotationalInertia,

            double wheelsCoefficientOfFriction) {

        check(driveGearRatio, 4, 24, "drive gear ratio", "times reduction");

        check(steerGearRatio, 6, 50, "steer gear ratio", "times reduction");

        check(driveFrictionVoltage.in(Volts), 0.01, 0.35, "drive friction voltage", "volts");

        check(steerFrictionVoltage.in(Volts), 0.01, 0.6, "steer friction voltage", "volts");

        check(wheelRadius.in(Inches), 1, 3.2, "drive wheel radius", "inches");

        check(

                steerRotationalInertia.in(KilogramSquareMeters), 0.005, 0.06, "steer rotation inertia", "kg * m^2");

        check(wheelsCoefficientOfFriction, 0.6, 1.9, "tire coefficient of friction", "");



        this.driveMotorConfigs =

                new SimMotorConfigs(driveMotorModel, driveGearRatio, KilogramSquareMeters.zero(), driveFrictionVoltage);

        this.steerMotorConfigs =

                new SimMotorConfigs(steerMotorModel, steerGearRatio, steerRotationalInertia, steerFrictionVoltage);

        DRIVE_GEAR_RATIO = driveGearRatio;

        STEER_GEAR_RATIO = steerGearRatio;

        WHEELS_COEFFICIENT_OF_FRICTION = wheelsCoefficientOfFriction;

        DRIVE_FRICTION_VOLTAGE = driveFrictionVoltage;

        WHEEL_RADIUS = wheelRadius;

    }

    

    @Override

    public SwerveModuleSimulation get() {

        return new SwerveModuleSimulation(
            driveMotorConfigs, 
            steerMotorConfigs, 
            DRIVE_GEAR_RATIO, 
            STEER_GEAR_RATIO, 
            WHEEL_RADIUS, 
            WHEELS_COEFFICIENT_OF_FRICTION
        );
        

    }



    public double getGrippingForceNewtons(double gravityForceOnModuleNewtons) {

        return gravityForceOnModuleNewtons * WHEELS_COEFFICIENT_OF_FRICTION;

    }



    public LinearVelocity maximumGroundSpeed() {

        return MetersPerSecond.of(

                driveMotorConfigs.freeSpinMechanismVelocity().in(RadiansPerSecond) * WHEEL_RADIUS.in(Meters));

    }



    public Force getTheoreticalPropellingForcePerModule(Mass robotMass, int modulesCount, Current statorCurrentLimit) {

        final double

                maxThrustNewtons =

                        driveMotorConfigs.calculateTorque(statorCurrentLimit).in(NewtonMeters)

                                / WHEEL_RADIUS.in(Meters),

                maxGrippingNewtons = 9.8 * robotMass.in(Kilograms) / modulesCount * WHEELS_COEFFICIENT_OF_FRICTION;



        return Newtons.of(Math.min(maxThrustNewtons, maxGrippingNewtons));

    }



    public LinearAcceleration maxAcceleration(Mass robotMass, int modulesCount, Current statorCurrentLimit) {

        return getTheoreticalPropellingForcePerModule(robotMass, modulesCount, statorCurrentLimit)

                .times(modulesCount)

                .div(robotMass);

    }

}

