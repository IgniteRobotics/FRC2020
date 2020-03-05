/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sorter;

public class RunSorter extends CommandBase {
  private final Sorter m_sorter; 
  /**
   * Creates a new RunSorter.
   */
  public RunSorter(double speed, Sorter s) {
    m_sorter = s;
    m_sorter.sorterSpeed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_sorter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_sorter.inSorter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_sorter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
