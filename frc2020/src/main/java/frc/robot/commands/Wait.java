/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase {
  /**
   * Creates a new Wait.
   */
  private double timeToWait = 0.0;
  private double targetTime;

  public Wait(double timeMS) {
    // Use addRequirements() here to declare subsystem dependencies.

    timeToWait = timeMS;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetTime = System.currentTimeMillis() + timeToWait;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(System.currentTimeMillis() >= targetTime) {
      this.end(false);
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
