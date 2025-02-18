// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
//import frc.robot.commands.ClimbMaunal;
//import frc.robot.commands.DriveFast;
//import frc.robot.commands.DriveSlow;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GoToCoral1;
import frc.robot.commands.GoToCoral2;
import frc.robot.commands.GoToCoral3;
import frc.robot.commands.GoToCoral4;
import frc.robot.commands.GoToNet;
import frc.robot.commands.GoToProcessor;
import frc.robot.commands.GyroReset;
import frc.robot.commands.GyroSwerveDriveCommand;
import frc.robot.commands.IntakeCoral;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
//import frc.robot.commands.IntakeRun;
//import frc.robot.commands.IntakeStop;
//import frc.robot.commands.ManualCommand;
//import frc.robot.commands.ManualIntake;
//import frc.robot.commands.ManualShoot;
//import frc.robot.commands.MaxShoot;
//import frc.robot.commands.Outake;
//import frc.robot.commands.ShootAmp;
//import frc.robot.commands.ShootSpeaker;
//import frc.robot.commands.ShooterIntake;
//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
//import frc.robot.subsystems.FlywheelShooter;
import frc.robot.subsystems.GyroSwerveDrive;
//import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotStates;

import java.io.ObjectInputStream.GetField;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCoral;

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
  public final Elevator m_Coral1 = new Elevator();
  public final Elevator m_Coral2 = new Elevator();
  public final Elevator m_Coral3 = new Elevator();
  public final Elevator m_Coral4 = new Elevator();
  public final Elevator m_Processor = new Elevator();
  public final Elevator m_Net = new Elevator();
  public final CoralIntake m_intake = new CoralIntake();
  public final RobotStates m_robotStates = new RobotStates();
  public final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  public final GyroSwerveDrive m_gyroSwerveDrive = new GyroSwerveDrive(m_robotStates, m_gyro);
  //public final Climber m_climber = new Climber();

  public SendableChooser<String> chooserFirst = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  XboxController driveStick = new XboxController(2);
  XboxController controlStick = new XboxController(1);

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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new JoystickButton(controlStick, Button.kRightBumper.value).whileTrue(new IntakeRun(m_intake, m_robotStates));
    //new JoystickButton(controlStick, Button.kRightBumper.value).whileFalse(new IntakeStop(m_intake));
    new JoystickButton(controlStick, Button.kLeftBumper.value).whileTrue(new IntakeCoral(m_intake));
    new JoystickButton(driveStick, Button.kX.value).whileTrue(new GyroReset(m_gyroSwerveDrive));
    new JoystickButton(controlStick, Button.kA.value).whileTrue(new GoToCoral1(m_Coral1));
    new JoystickButton(controlStick, Button.kB.value).whileTrue(new GoToCoral2(m_Coral2));
    new JoystickButton(controlStick, Button.kY.value).whileTrue(new GoToCoral3(m_Coral3));
    new JoystickButton(controlStick, Button.kRightBumper.value).whileTrue(new GoToCoral4(m_Coral4));
    new JoystickButton(controlStick, Button.kLeftStick.value).whileTrue(new GoToProcessor(m_Processor));
    new JoystickButton(controlStick, Button.kRightStick.value).whileTrue(new GoToNet(m_Net));
    //new JoystickButton(controlStick, 7).whileTrue(new MaxShoot(m_shooter, m_intake, m_robotStates));
    //new JoystickButton(driveStick, Button.kRightBumper.value).whileTrue(new DriveFast(m_robotStates));
    //new JoystickButton(driveStick, Button.kRightBumper.value).whileFalse(new DriveSlow(m_robotStates));
    ////new JoystickButton(controlStick, Button.kA.value).whileTrue(new ShooterIntake(m_shooter));
    //new JoystickButton(controlStick, Button.kY.value).whileTrue(new ManualIntake(m_intake));
    //new JoystickButton(controlStick, Button.kB.value).whileTrue(new ManualShoot(m_shooter, m_intake, m_robotStates));
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
    .withWidget(BuiltInWidgets.kTextView)
    .withPosition(8, 5)
;

    Shuffleboard.getTab("Test")
    .add("Auton", chooserFirst)
    .withWidget(BuiltInWidgets.kSplitButtonChooser)
    .withSize(3,1)
    .withPosition(0,0);
    Shuffleboard.getTab("Test");
    /*.add("Note", m_intake.gotNote)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withSize(2,3)
    .withPosition(9, 2);
    /*Shuffleboard.getTab("Test")
    .addCamera("Limelight", "limelight", null)
    .withSize(4, 3)
    .withPosition(0, 1);*/
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
