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

public class Sorter extends SubsystemBase {
  /**
   * Creates a new Sorter.
   */

   public WPI_VictorSPX rightMotor;
   public WPI_VictorSPX leftMotor;

   private double intakePower = .4;



  public Sorter(int rightSorterMotorCanID, int leftSorterMotorCanID)
  {
    rightMotor = new WPI_VictorSPX(rightSorterMotorCanID);
    leftMotor = new WPI_VictorSPX(leftSorterMotorCanID);
    rightMotor.setNeutralMode(NeutralMode.Brake);
    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setInverted(false);
    leftMotor.setInverted(true);

  }

  public void Run() {
    rightMotor.set(-intakePower);
    leftMotor.set(intakePower);
  }

  public void Stop() {
    rightMotor.set(0);
    leftMotor.set(0);
  }

  public void Eject() {
    rightMotor.set(intakePower);
    leftMotor.set(-intakePower);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
