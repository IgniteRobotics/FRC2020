/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Sorter extends SubsystemBase {
  private final WPI_VictorSPX sorterMotor;
  private final DigitalInput sorterBeamBreak;

  public double sorterSpeed = 0.5;

  /**
   * Creates a new Sorter.
   */
  public Sorter() {
    sorterMotor = new WPI_VictorSPX(Constants.kSorterMotorPort);
    sorterMotor.setInverted(false);
    sorterMotor.setNeutralMode(NeutralMode.Brake);

    sorterBeamBreak = new DigitalInput(Constants.kSorterSensorPort);
  }

  public void inSorter() {
    sorterMotor.set(ControlMode.PercentOutput, sorterSpeed);
  }

  public void stop() {
    sorterMotor.set(ControlMode.PercentOutput, sorterSpeed);
  }

  public boolean getSensor() {
    return sorterBeamBreak.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
