/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubSystem;

public class OdomotryDrive extends CommandBase {
  private final DriveTrainSubSystem drivtrainsubsystem; 
  DifferentialDriveOdometry odoDrive;
  /**
   * Creates a new OdomotryDrive.
   */
  public OdomotryDrive(DriveTrainSubSystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivtrainsubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DifferentialDriveOdometry odoDrive = new DifferentialDriveOdometry(Rotation2d.fromDegrees(drivtrainsubsystem.getNavXAngle()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    odoDrive.update(Rotation2d.fromDegrees(drivtrainsubsystem.getNavXAngle()), leftDistanceMeters, rightDistanceMeters)
    System.out.println(odoDrive.getPoseMeters());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
