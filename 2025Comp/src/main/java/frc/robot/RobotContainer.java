// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.GyroSwerveDrive;
import frc.robot.subsystems.RobotStates;


import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Elevator m_elevator = new Elevator();
  public final CoralIntake m_coral = new CoralIntake();
  public final CoralAngle m_angle = new CoralAngle(); 
  public final AlgaeIntake m_algae = new AlgaeIntake();
  public final Climber m_climber = new Climber();
  public final TrapMotor m_trap = new TrapMotor();
  public final RobotStates m_robotStates = new RobotStates();
  public final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  public final GyroSwerveDrive m_gyroSwerveDrive = new GyroSwerveDrive(m_robotStates, m_gyro);

  private final Command m_moveForward = new AutoMoveOffLine(m_gyroSwerveDrive);
  private final Command m_moveAndScoreLow = new AutoMoveAndScoreLow(m_gyroSwerveDrive, m_elevator, m_angle, m_coral);
  private final Command m_moveAndSetHigh = new AutoMoveAndSetHigh(m_gyroSwerveDrive, m_elevator, m_angle);

  public SendableChooser<Command> chooserFirst = new SendableChooser<>();

  // Joystick calls CommandPS4Controller, CommandJoystick, CommandXboxController, CommandGenericHID.
  // can remove "command" to get just a joystick instance.  unsure what all differences are appears that command allows for .button.whiletrue calls
  // where as non command makes us have to create a new button instance.
  XboxController driveStick = new XboxController(3);
  CommandGenericHID buttonBoard1 = new CommandGenericHID(1);
  CommandGenericHID buttonBoard2 = new CommandGenericHID(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //*

   /* m_climber.setDefaultCommand(
      new ClimbMaunal(m_climber, () -> -controlStick.getRightY(), () -> controlStick.getLeftY())
    );*/

    // Configure the trigger bindings
    NamedCommands.registerCommand("gyroReset", new GyroReset(m_gyroSwerveDrive));

    chooserFirst.setDefaultOption("DriveForward", m_moveForward);
    chooserFirst.addOption("Drive and Score", m_moveAndScoreLow);
    chooserFirst.addOption("Move and set to L4", m_moveAndSetHigh);

    SmartDashboard.putData(chooserFirst);

    setupShuffleboard();
    
    configureBindings();
  }
  
  //now fixed

  /**
   *Use this method to define your trigger->command mappings. Triggers can be created via the-
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new JoystickButton(driveStick, Button.kX.value).whileTrue(new GyroReset(m_gyroSwerveDrive));
    new JoystickButton(driveStick, Button.kLeftBumper.value).whileTrue(new AutoAlignLeft(m_gyroSwerveDrive, Constants.AUTO_LEFT_OFFSET, m_robotStates));
    buttonBoard1.button(1).onTrue(new GoToProcessor(m_elevator));
    buttonBoard1.button(2).onTrue(new GoToCoral1Angle(m_elevator, m_angle));
    buttonBoard1.button(3).onTrue(new GoToCoral2Angle(m_elevator, m_angle));
    buttonBoard1.button(4).onTrue(new GoToIntakeAngle(m_elevator, m_angle));
    buttonBoard1.button(5).onTrue(new GoToCoral3Angle(m_elevator, m_angle));
    buttonBoard1.button(6).onTrue(new GoToCoral4Angle(m_elevator, m_angle));
    buttonBoard1.button(7).onTrue(new GoToNet(m_elevator));

    buttonBoard1.axisLessThan(0, -0.5).whileTrue(new ManualElevatorUp(m_elevator));
    buttonBoard1.axisGreaterThan(0, 0.5).whileTrue(new ManualElevatorDown(m_elevator));


    buttonBoard2.axisGreaterThan(0,0.5).whileTrue(new ManualAngleUp(m_angle));
    buttonBoard2.axisLessThan(0,-0.5).whileTrue(new ManualAngleDown(m_angle));
    buttonBoard2.axisGreaterThan(1,0.5).whileTrue(new TrapClose(m_trap));
    buttonBoard2.axisLessThan(1,-0.5).whileTrue(new TrapOpen(m_trap));
    buttonBoard2.button(3).whileTrue(new IntakeCoral(m_coral));
    buttonBoard2.button(4).whileTrue(new OuttakeCoral(m_coral));
    buttonBoard2.button(1).whileTrue(new IntakeAlgae(m_algae));
    buttonBoard2.button(2).whileTrue(new OuttakeAlgae(m_algae));
    buttonBoard2.button(5).whileTrue(new ClimbUp(m_climber));
    buttonBoard2.button(5).whileFalse(new CilmbStop(m_climber));
    //new POVButton(buttonBoard, 0).whileTrue(new ClimbUp(m_climber))
  }

  
  public void setupShuffleboard(){
    Shuffleboard.getTab("Test")
    .add("Gyro", m_gyro.getAngle(m_gyro.getYawAxis()))
    .withWidget(BuiltInWidgets.kGyro)
    .withSize(2, 2)
    .withPosition(8, 0);

    /*Shuffleboard.getTab("Test")
    .add("Sim", m_gyroSwerveDrive.getPose())
    .withSize(3,3)
    .withWidget(BuiltInWidgets.kField)
    .withPosition(8, 5);*/

    Shuffleboard.getTab("Test")
    .add("Auton", chooserFirst)
    .withWidget(BuiltInWidgets.kSplitButtonChooser)
    .withSize(3,1)
    .withPosition(0,0);

    Shuffleboard.getTab("Test")
    .add("Elevator", m_elevator.elevatorEncoder.getPosition())
    .withWidget(BuiltInWidgets.kTextView)
    .withSize(2,1);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return chooserFirst.getSelected();

  }
}
