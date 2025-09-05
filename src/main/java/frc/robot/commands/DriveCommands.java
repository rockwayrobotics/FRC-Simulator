// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

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
