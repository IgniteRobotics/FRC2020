/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoForward extends CommandBase {
  /**
   * Creates a new AutoForward.
   */
  private final DriveTrain m_driveTrain;
  private final double m_timeout; //in millis
  private double targetTime;
  
  public AutoForward(DriveTrain driveTrain, double timeout) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    m_timeout = timeout;
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.stop();
    targetTime = System.currentTimeMillis() + m_timeout;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(System.currentTimeMillis() >= targetTime) {
      end(false);
    }
    else {
      m_driveTrain.setOpenLoopLeft(0.25);
      m_driveTrain.setOpenLoopRight(0.25);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
