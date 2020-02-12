/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.Shooter;

public class Velocityshoot extends CommandBase {

  private final Shooter m_shooter;
  /**
   * Creates a new Velocityshoot.
   */
  public Velocityshoot(Shooter _shooter) {
    
    addRequirements(_shooter);
    this.m_shooter = _shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("velocity_1", 0);
    SmartDashboard.putNumber("power_1", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Velocity = SmartDashboard.getNumber("velocity_1", 0);
    int kSlotIdx = (int) SmartDashboard.getNumber("kSlotIdx", 0);
    int PIDLoopIdx = (int) SmartDashboard.getNumber("kSlotIdx", 0);
    int kTimeoutMs = (int) SmartDashboard.getNumber("kSlotIdx", 30);
    double kP = SmartDashboard.getNumber("kP", .25);
    double kI = SmartDashboard.getNumber("kP", 0);
    double kD = SmartDashboard.getNumber("kP", 0);    
    double kF = SmartDashboard.getNumber("kP", 1023.0/7200.0);
    m_shooter.configuration(kSlotIdx, PIDLoopIdx, kTimeoutMs, kP, kI, kD, kF);
    m_shooter.setVelocity(Velocity);
    SmartDashboard.putNumber("velocity1", m_shooter.getvelocity());
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
