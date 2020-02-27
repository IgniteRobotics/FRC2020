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
public class kick extends CommandBase {
  private static Spindexer kicker;
  private FireControl m_fireControl;
  /**
   * Creates a new kick.
   */
  public kick(Spindexer kicker) {
    addRequirements(kicker);
    // Use addRequirements() here to declare subsystem dependencies.
    m_fireControl = FireControl.getInstance();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //kicker.toggleKicker();
    kicker.spinWheel();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO:  this needs a way to test if we're at full speed, or a delay, or something.
    this.m_fireControl.setKickerVelocityStatus(true);
    //TODO:  this should likely be "extend" instead of "toggle"
    if (this.m_fireControl.readyToFire()){
      kicker.toggleKicker();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    kicker.stopWheel();
    kicker.toggleKicker();
    m_fireControl.setKickerVelocityStatus(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
