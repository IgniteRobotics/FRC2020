/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spindexer;
import frc.robot.util.FireControl;
public class spindex extends CommandBase {
  private static Spindexer m_spindexer;
  private FireControl m_fireControl;
  /**
   * Creates a new kick.
   */
  public spindex(Spindexer spindexer) {
    addRequirements(spindexer);
    this.m_spindexer = spindexer;
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_fireControl = FireControl.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Only spin if we're ready to fire
    //TODO:  this breaks intake.  it's just an example for the shooter
    if (m_fireControl.readyToFire()){
      m_spindexer.spinClockwise();
    } 
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
