/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

public class AutoShootNMove extends CommandBase {
  /**
   * Creates a new AutoShootNMove.
   */
  private final DriveTrain m_driveTrain;
  private final Kicker m_kicker;
  private final Shooter m_shooter;
  private final Spindexer m_spindexer;
  private final Intake m_intake;

  public AutoShootNMove(DriveTrain driveTrain, Kicker kicker, Shooter shooter, Spindexer spindexer, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    m_kicker = kicker;
    m_shooter = shooter;
    m_spindexer = spindexer;
    m_intake = intake;
    addRequirements(m_driveTrain);
    addRequirements(m_kicker);
    addRequirements(m_shooter);
    addRequirements(m_spindexer);
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.stop();
    m_kicker.stopKicker();
    m_shooter.setpower(0);
    m_spindexer.stop();
    m_intake.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
