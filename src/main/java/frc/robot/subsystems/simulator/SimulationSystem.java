package frc.robot.subsystems.simulator;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

public class SimulationSystem extends SubsystemBase {
  private Field2d field = new Field2d();
  Drive drive;

  public SimulationSystem(Drive drive) {
    this.drive = drive;
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    field.setRobotPose(drive.getPose());
  }
}
