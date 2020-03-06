/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spindexer;

public class SpinSpindexerToNext extends CommandBase {
  private final Spindexer m_spindexer;
  private final int encoderTicksPerFifthRev = -12000;
  private double currentTime;
  /**
   * Creates a new SpinSpindexerToNext.
   */
  public SpinSpindexerToNext(Spindexer spindexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_spindexer = spindexer;
    addRequirements(m_spindexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_spindexer.stop();
    m_spindexer.configPIDProfile(0, 0, 1, 0, 0);
    m_spindexer.resetEncoder();
    currentTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_spindexer.setMotionMagicPosition(encoderTicksPerFifthRev);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_spindexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_spindexer.isMotionMagicDone() || System.currentTimeMillis() > currentTime + 1000;
  }
}
