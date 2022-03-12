// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants.Arm;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.DigitalInput;


public class Rotate extends CommandBase {

	DigitalInput firstBarRightLimitSwitch, secondBarRightLimitSwitch, thirdBarRightLimitSwitch, firstBarLeftLimitSwitch, secondBarLeftLimitSwitch, thirdBarLeftLimitSwitch;

	private final Climber m_climber;
	private final int m_direction;
	private final Arm m_arm;
 

  public Rotate(Climber climber, Arm arm, int direction ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_direction = direction;
    m_arm = arm;
    addRequirements(m_climber);

    firstBarRightLimitSwitch = new DigitalInput(ClimberConstants.firstBarRightLimitSwitch);
    secondBarRightLimitSwitch = new DigitalInput(ClimberConstants.secondBarRightLimitSwitch);
    thirdBarRightLimitSwitch = new DigitalInput(ClimberConstants.thirdBarRightLimitSwitch);
    firstBarLeftLimitSwitch = new DigitalInput(ClimberConstants.firstBarLeftLimitSwitch);
    secondBarLeftLimitSwitch = new DigitalInput(ClimberConstants.secondBarLeftLimitSwitch);
    thirdBarLeftLimitSwitch = new DigitalInput(ClimberConstants.thirdBarLeftLimitSwitch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      //m_climber.SetArmAngle(360 * m_direction, m_arm);
    //m_climber.Start(m_direction, m_arm);
    if(firstBarRightLimitSwitch.get() || firstBarLeftLimitSwitch.get()){
      m_climber.MediumTraverseRelease();
    }else{
      m_climber.MediumTraverseGrab();
    }
    if(secondBarRightLimitSwitch.get() || secondBarLeftLimitSwitch.get()){
      m_climber.HighRelease();
    }else{
      m_climber.HighGrab();
    }
    if(thirdBarRightLimitSwitch.get() || thirdBarLeftLimitSwitch.get()){
      m_climber.MediumTraverseRelease();
    }else{
      m_climber.MediumTraverseGrab();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.Stop(m_arm);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
