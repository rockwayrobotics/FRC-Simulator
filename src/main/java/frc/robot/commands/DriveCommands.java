// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    switch (Constants.level) {
      case 1:
        return frc.robot.commands.level1.DriveCommands.joystickDrive(
            drive, xSupplier, ySupplier, omegaSupplier);
      case 2:
        return frc.robot.commands.level2.DriveCommands.joystickDrive(
            drive, xSupplier, ySupplier, omegaSupplier);
      case 3:
        return frc.robot.commands.level3.DriveCommands.joystickDrive(
            drive, xSupplier, ySupplier, omegaSupplier);
      case 4:
        return frc.robot.commands.level3.DriveCommands.joystickDrive(
            drive, xSupplier, ySupplier, omegaSupplier);
      case 5:
        return frc.robot.commands.level3.DriveCommands.joystickDrive(
            drive, xSupplier, ySupplier, omegaSupplier);
      default:
        return frc.robot.commands.real.DriveCommands.joystickDrive(
            drive, xSupplier, ySupplier, omegaSupplier);
    }
  }

  public static Command driveForward(Drive drive) {
    switch (Constants.level) {
      case 4:
        return frc.robot.commands.level4.DriveCommands.driveForward(drive);
      default:
        return null;
    }
  }

  public static Command slowModeDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      double scale) {
    switch (Constants.level) {
      case 5:
        return frc.robot.commands.level5.DriveCommands.slowModeDrive(
            drive, xSupplier, ySupplier, omegaSupplier, scale);
      default:
        return null;
    }
  }
}
