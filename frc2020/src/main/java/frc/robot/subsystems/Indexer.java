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

public class Indexer extends SubsystemBase {

  public WPI_VictorSPX indexerMotor;

  private double indexerMotorPower = 0.2;

  public Indexer (int indexerMotorCanID)
  { 
    indexerMotor = new WPI_VictorSPX(indexerMotorCanID);
    indexerMotor.setNeutralMode(NeutralMode.Brake);    

  }
  
  public void TurnClockwise() {
    indexerMotor.set(indexerMotorPower);

  }

  public void TurnCounterClockwise() {
    indexerMotor.set(-indexerMotorPower);
  }

  public void Stop() {
    indexerMotor.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
