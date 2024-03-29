/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Spindexer;

public class SpinNKick extends CommandBase {
  /**
   * Creates a new SpinNKick.
   */
  private final Spindexer m_spindexer;
  private final Kicker m_kicker;
  
  public SpinNKick(Spindexer sd, Kicker k) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_spindexer = sd;
    m_kicker = k;
    addRequirements(m_spindexer);
    addRequirements(m_kicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_spindexer.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_spindexer.spinCounterClockwise(0.15);
    m_kicker.spinKickerWheel(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_spindexer.stop();
    m_kicker.stopKicker();
    // m_spindexer.toggleKicker();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
