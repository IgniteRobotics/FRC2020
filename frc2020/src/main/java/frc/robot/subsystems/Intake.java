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

public class Intake extends SubsystemBase {

  public WPI_VictorSPX rollerMotor;

  private double intakePower = .8;
  

  public Intake (int rollerMotorCanID)
  {
    rollerMotor = new WPI_VictorSPX(rollerMotorCanID);
    rollerMotor.setNeutralMode(NeutralMode.Brake);
    rollerMotor.setInverted(false);

  }

  public void Run() {
    rollerMotor.set(intakePower);
  }

  public void Stop() {
    rollerMotor.set(0);
  }

  public void Eject(){
    rollerMotor.set(-intakePower);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
