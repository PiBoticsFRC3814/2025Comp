// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralAngle;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GyroSwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoMoveAndScore extends SequentialCommandGroup {
  /** Creates a new AutoMoveAndScore. */
  public AutoMoveAndScore(GyroSwerveDrive drive, Elevator coral, CoralAngle angle, CoralIntake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoMoveOffLine(drive), new GoToCoral2Angle(coral, angle), new ShortWait(), new OuttakeCoral(intake));
  }
}
