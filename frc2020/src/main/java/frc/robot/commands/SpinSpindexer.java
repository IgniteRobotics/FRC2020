/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spindexer;

public class SpinSpindexer extends CommandBase {
  private final Spindexer m_spindexer;

  private boolean isCounterClockWise;
  /**
   * Creates a new SpinSpindexer.
   */
  public SpinSpindexer(boolean icc, double speed, Spindexer sd) {
    m_spindexer = sd;
    isCounterClockWise = icc;
    m_spindexer.spindexerSpeed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_spindexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_spindexer.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isCounterClockWise) {
      m_spindexer.spinCounterClockwise();
    }
    else {
      m_spindexer.spinClockwise();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_spindexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}