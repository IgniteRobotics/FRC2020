/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final WPI_TalonSRX shooterTalon;
  private final WPI_VictorSPX shooterVictor;
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    shooterTalon = new WPI_TalonSRX(Constants.kShooterTalonMotorPort);
    shooterVictor = new WPI_VictorSPX(Constants.kShooterVictorMotorPort);
    shooterVictor.setInverted(true);
  }

  public void shoot(){
    shooterTalon.set(ControlMode.PercentOutput, .5);// dummy command
    shooterVictor.set(ControlMode.PercentOutput, .5);
    
    System.out.println(shooterTalon.getSelectedSensorPosition());
  }

  public void end(){
    
    shooterTalon.set(ControlMode.PercentOutput, .0);//i learned from your mistake
    shooterVictor.set(ControlMode.PercentOutput, .0);// dummy code
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
