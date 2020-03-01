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
  private double m_speed;
  /**
   * Creates a new Velocityshoot.
   */
  public Velocityshoot(double speed, Shooter _shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(_shooter);
    this.m_shooter = _shooter;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int kSlotIdx = (int) SmartDashboard.getNumber("kSlotIdx (Shooter)", 0);
    int PIDLoopIdx = (int) SmartDashboard.getNumber("PIDLoopIdx (Shooter)", 0);
    int kTimeoutMs = (int) SmartDashboard.getNumber("kTimeoutMs (Shooter)", 30);
    double kP = SmartDashboard.getNumber("kP shooter", .25);
    double kI = SmartDashboard.getNumber("kI shooter", 0);
    double kD = SmartDashboard.getNumber("kD shooter", 0);    
    double kF = SmartDashboard.getNumber("kF shooter", 1023.0/7200.0);
    m_shooter.configuration(kSlotIdx, PIDLoopIdx, kTimeoutMs, kP, kI, kD, kF);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_shooter.setVelocity(rpmtotics(m_speed));
    SmartDashboard.putNumber("velocityvalue", m_shooter.getvelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setVelocity(0);
  }

  private double rpmtotics(double m_speed) {
    return ((m_speed / 60) * 8192) / 10;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
