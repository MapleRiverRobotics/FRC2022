// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private CANSparkMax shooterMotorOne;
  private CANSparkMax shooterMotorTwo;

  public Shooter() {
    shooterMotorOne = new CANSparkMax(ShooterConstants.ShooterMotorOneId, MotorType.kBrushless);
    shooterMotorTwo = new CANSparkMax(ShooterConstants.ShooterMotorTwoId, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Start(double speed){
    shooterMotorOne.set(speed/100);
  }

  public void Stop() {
    shooterMotorOne.set(0);
    shooterMotorTwo.set(0);
  }
}
