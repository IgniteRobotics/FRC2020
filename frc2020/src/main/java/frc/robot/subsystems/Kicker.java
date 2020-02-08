/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kicker extends SubsystemBase {

  public WPI_VictorSPX kickerMotor;

  private double kickerMotorPower = 0.8;

  public Kicker (int kickerMotorCanID)
  { 
    kickerMotor = new WPI_VictorSPX(kickerMotorCanID);
    kickerMotor.setNeutralMode(NeutralMode.Brake); 

  }
  
  public void Run() {
    kickerMotor.set(kickerMotorPower);
  }

  public void Stop() {
    kickerMotor.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
