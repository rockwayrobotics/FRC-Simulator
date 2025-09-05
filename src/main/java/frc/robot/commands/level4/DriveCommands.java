// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands.level4;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    return Commands.run(
        () -> {
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  xSupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec(),
                  ySupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec(),
                  omegaSupplier.getAsDouble() * drive.getMaxAngularSpeedRadPerSec());
          drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
        },
        drive);
  }

  public static Command driveForward(Drive drive) {
    return new RunCommand(
            () -> {
              double max = drive.getMaxLinearSpeedMetersPerSec();
              ChassisSpeeds speeds = new ChassisSpeeds(0, max * 0.5, 0);
              drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
            },
            drive)
        .withTimeout(2.0);
  }
}
