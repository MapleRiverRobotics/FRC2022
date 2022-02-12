// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ShooterConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Shooter. */

  DoubleSolenoid mediumTraverseValve = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.MediumTraverseGrabId, ClimberConstants.MediumTraverseReleaseId);
  DoubleSolenoid highValve = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.HighValveGrabId, ClimberConstants.HighValveReleaseId);

  public void MediumTraverseRelease() {
    mediumTraverseValve.set(Value.kReverse);
  }

  public void MediumTraverseGrab() {
    mediumTraverseValve.set(Value.kForward);
  }

  public void HighRelease() {
    highValve.set(Value.kReverse);
  }

  public void HighGrab() {
    highValve.set(Value.kForward);
  }

  public Climber() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Stop() {
    highValve.set(Value.kOff);
    mediumTraverseValve.set(Value.kOff);
  }
}
