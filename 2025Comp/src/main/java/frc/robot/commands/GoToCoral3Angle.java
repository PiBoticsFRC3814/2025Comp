// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.GoToCoral1;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralAngle;
import frc.robot.commands.AngleMiddle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToCoral3Angle extends ParallelCommandGroup {
  /** Creates a new ElevatorAndAngle. */
  public GoToCoral3Angle(Elevator coral, CoralAngle angle) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new GoToCoral3(coral),new AngleMiddle(angle));

  }
}
