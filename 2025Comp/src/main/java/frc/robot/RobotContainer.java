// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GyroSwerveDrive;
import frc.robot.subsystems.RobotStates;

import java.io.ObjectInputStream.GetField;
import java.lang.ModuleLayer.Controller;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  //public final FlywheelShooter m_shooter = new FlywheelShooter();
  public final Elevator m_elevator = new Elevator();
  public final CoralIntake m_coral = new CoralIntake();
  public final CoralAngle m_angle = new CoralAngle(); 
  public final AlgaeIntake m_algae = new AlgaeIntake();
  public final Climber m_climber = new Climber();
  public final RobotStates m_robotStates = new RobotStates();
  public final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  public final GyroSwerveDrive m_gyroSwerveDrive = new GyroSwerveDrive(m_robotStates, m_gyro);
  //public final Climber m_climber = new Climber();

  public SendableChooser<String> chooserFirst = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  XboxController driveStick = new XboxController(2);
  //XboxController controlStick = new XboxController(1);
  GenericHID buttonBoard = new GenericHID(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //*

   /* m_climber.setDefaultCommand(
      new ClimbMaunal(m_climber, () -> -controlStick.getRightY(), () -> controlStick.getLeftY())
    );*/

    //m_shooter.setDefaultCommand(new ManualCommand(m_shooter, () -> controlStick.getLeftTriggerAxis(), () -> controlStick.getRightTriggerAxis()));
    // Configure the trigger bindings
    double speed = 1300;//SmartDashboard.getNumber("Speed", 0.0);
    //m_shooter.setDefaultCommand(new ShootWithSlider(m_shooter, () -> speed, () -> m_driverController.getThrottle()));

    //NamedCommands.registerCommand("intakeRun", new IntakeRun(m_intake, m_robotStates));
    //NamedCommands.registerCommand("intakeStop", new IntakeStop(m_intake));
    //NamedCommands.registerCommand("SpeakerFire", new ShootSpeaker(m_shooter, m_intake, m_robotStates));
    //NamedCommands.registerCommand("shootAmp", new ShootAmp(m_shooter, m_intake, m_robotStates));
    NamedCommands.registerCommand("gyroReset", new GyroReset(m_gyroSwerveDrive));

    chooserFirst.setDefaultOption("Test", "Test Auto");
    /*chooserFirst.addOption("Amp Side", "Left Auto");
    chooserFirst.addOption("Stage Side", "Right Auto");
    chooserFirst.addOption("Shoot test", "New Auto");
    chooserFirst.addOption("4 Note Front", "YESSSSS");
    chooserFirst.addOption("SCRAMMMM", "GetOut");
    chooserFirst.addOption("Front Left", "Front Left");
    chooserFirst.addOption("Front Right", "Front Right");
    chooserFirst.addOption("Front Middle", "Front Middle");
    //setupShuffleboard();
    //*/

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
    new JoystickButton(buttonBoard, 1).whileTrue(new GoToProcessor(m_elevator));
    new JoystickButton(buttonBoard, 2).whileTrue(new GoToCoral1Angle(m_elevator, m_angle));
    new JoystickButton(buttonBoard, 3).whileTrue(new GoToCoral2Angle(m_elevator, m_angle));
    new JoystickButton(buttonBoard, 4).whileTrue(new GoToIntakeAngle(m_elevator, m_angle));
    new JoystickButton(buttonBoard, 5).whileTrue(new GoToCoral3Angle(m_elevator, m_angle));
    new JoystickButton(buttonBoard, 6).whileTrue(new GoToCoral4Angle(m_elevator, m_angle));
    new JoystickButton(buttonBoard, 7).whileTrue(new GoToNet(m_elevator));
    new JoystickButton(buttonBoard, 8).whileTrue(new ManualElevatorDown(m_elevator));
    new JoystickButton(buttonBoard, 9).whileTrue(new ManualElevatorUp(m_elevator));
    new JoystickButton(buttonBoard, 10).whileTrue(new ManualAngleDown(m_angle));
    new JoystickButton(buttonBoard, 11).whileTrue(new ManualAngleUp(m_angle));
    new JoystickButton(buttonBoard, 12).whileTrue(new IntakeCoral(m_coral));
    new JoystickButton(buttonBoard, 13).whileTrue(new OuttakeCoral(m_coral));
    new JoystickButton(buttonBoard, 14).whileTrue(new IntakeAlgae(m_algae));
    new JoystickButton(buttonBoard, 15).whileTrue(new OuttakeAlgae(m_algae));
    new JoystickButton(buttonBoard, 16).whileTrue(new ClimbUp(m_climber));


  }

  
  public void setupShuffleboard(){
    Shuffleboard.getTab("Test")
    .add("Gyro", m_gyro.getAngle(m_gyro.getYawAxis()))
    .withWidget(BuiltInWidgets.kGyro)
    .withSize(2, 2)
    .withPosition(8, 0);

    Shuffleboard.getTab("Test")
    .add("Sim", m_gyroSwerveDrive.getPose())
    .withSize(3,3)
    .withWidget(BuiltInWidgets.kField)
    .withPosition(8, 5);

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
    m_robotStates.center = chooserFirst.getSelected() == "Center Auto";
    return new PathPlannerAuto(chooserFirst.getSelected());
  }
}
