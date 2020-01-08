/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDrive extends CommandBase {
  /**
   * Creates a new ArcadeDrive.
   */
  private static DriveTrain m_driveTrain;
  private static double m_throttle;
  private static double m_rotation;
  private static double m_deadband;

  public ArcadeDrive(DriveTrain driveTrain, double throttle, double rotation, double deadband) {
    m_driveTrain = driveTrain;
    m_throttle = throttle;
    m_rotation = rotation;
    m_deadband = deadband;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = m_throttle;
    double rotation = m_rotation;

    m_driveTrain.arcadeDrive(-throttle, rotation, m_deadband);
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
