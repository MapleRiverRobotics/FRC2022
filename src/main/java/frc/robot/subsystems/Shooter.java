// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private CANSparkMax shooterMotorOne;
  private CANSparkMax shooterMotorTwo;
  private SparkMaxPIDController m_pidControllerOne;
  private RelativeEncoder m_encoderOne;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  public Shooter() {
    //shooterMotorOne = new CANSparkMax(ShooterConstants.ShooterMotorOneId, MotorType.kBrushless);
    shooterMotorTwo = new CANSparkMax(ShooterConstants.ShooterMotorTwoId, MotorType.kBrushless);
    //shooterMotorTwo.follow(shooterMotorOne);

    // /**
    //  * In order to use PID functionality for a controller, a SparkMaxPIDController
    //  * object
    //  * is constructed by calling the getPIDController() method on an existing
    //  * CANSparkMax object
    //  */
    // m_pidControllerOne = shooterMotorOne.getPIDController();

    // // Encoder object created to display position values
    // m_encoderOne = shooterMotorOne.getEncoder();

    // // PID coefficients
    // kP = 6e-5;
    // kI = 0;
    // kD = 0;
    // kIz = 0;
    // kFF = 0.000015;
    // kMaxOutput = 1;
    // kMinOutput = -1;
    // maxRPM = 5000;

    // // set PID coefficients
    // m_pidControllerOne.setP(kP);
    // m_pidControllerOne.setI(kI);
    // m_pidControllerOne.setD(kD);
    // m_pidControllerOne.setIZone(kIz);
    // m_pidControllerOne.setFF(kFF);
    // m_pidControllerOne.setOutputRange(kMinOutput, kMaxOutput);

    // // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
  }

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    // // read PID coefficients from SmartDashboard
    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
    // double iz = SmartDashboard.getNumber("I Zone", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Max Output", 0);
    // double min = SmartDashboard.getNumber("Min Output", 0);

    // // if PID coefficients on SmartDashboard have changed, write new values to
    // // controller
    // if ((p != kP)) {
    //   m_pidControllerOne.setP(p);
    //   kP = p;
    // }
    // if ((i != kI)) {
    //   m_pidControllerOne.setI(i);
    //   kI = i;
    // }
    // if ((d != kD)) {
    //   m_pidControllerOne.setD(d);
    //   kD = d;
    // }
    // if ((iz != kIz)) {
    //   m_pidControllerOne.setIZone(iz);
    //   kIz = iz;
    // }
    // if ((ff != kFF)) {
    //   m_pidControllerOne.setFF(ff);
    //   kFF = ff;
    // }
    // if ((max != kMaxOutput) || (min != kMinOutput)) {
    //   m_pidControllerOne.setOutputRange(min, max);
    //   kMinOutput = min;
    //   kMaxOutput = max;
    // }

    // /**
    //  * PIDController objects are commanded to a set point using the 
    //  * SetReference() method.
    //  * 
    //  * The first parameter is the value of the set point, whose units vary
    //  * depending on the control type set in the second parameter.
    //  * 
    //  * The second parameter is the control type can be set to one of four 
    //  * parameters:
    //  *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
    //  *  com.revrobotics.CANSparkMax.ControlType.kPosition
    //  *  com.revrobotics.CANSparkMax.ControlType.kVelocity
    //  *  com.revrobotics.CANSparkMax.ControlType.kVoltage
    //  */
    // double setPoint = maxRPM;
    // m_pidControllerOne.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    
    // SmartDashboard.putNumber("SetPoint", setPoint);
    // SmartDashboard.putNumber("ProcessVariable", m_encoderOne.getVelocity());
  }

  public void Start(double speed) {
    shooterMotorTwo.set(speed / 100);
  }

  public void Stop() {
    shooterMotorTwo.set(0);
  }
}
