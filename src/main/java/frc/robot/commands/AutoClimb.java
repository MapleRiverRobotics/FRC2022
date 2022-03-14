/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants.Arm;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class AutoClimb extends CommandBase {
    private final Climber m_climber;
    private final int m_barNumber;
    public boolean barOneFinished;
    public static boolean barOneGrabbed = false;
    public boolean barTwoFinished, barTwoGrabbed;
    public boolean barThreeFinished;

  public AutoClimb(Climber climber, int barNumber) {
    m_climber = climber;
    m_barNumber = barNumber;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    barOneFinished = false;
    barTwoFinished = false;
    barThreeFinished = false;
    barTwoGrabbed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_barNumber == 1){
      climbFirstBar();
    }else if(m_barNumber == 2){
      climbSecondBar();
    }else if(m_barNumber == 3){
      climbThirdBar();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.Stop(Arm.Both);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void climbFirstBar(){
    if(barOneFinished == false){
      if(m_climber.IsFirstBarLeftLimitSwitchPressed() && m_climber.IsFirstBarRightLimitSwitchPressed()){
        barOneFinished = true;
        barOneGrabbed = true;
        m_climber.MediumTraverseGrab();
      }
    }
  }

  public void climbSecondBar(){
    if(barTwoFinished == false){
      //Check and grab bar 2
      if(m_climber.IsSecondBarLeftLimitSwitchPressed() && m_climber.IsSecondBarRightLimitSwitchPressed()){
        m_climber.HighGrab();
        barTwoGrabbed = true;
        m_climber.Stop(Arm.Both);
      }else{
        m_climber.Start(1, Arm.Both);
      }
    }
  }

  public void climbThirdBar(){
    //Release first bar
    if(barOneGrabbed == true){
      if(m_climber.IsFirstBarLeftLimitSwitchPressed() || m_climber.IsFirstBarRightLimitSwitchPressed()){
        m_climber.MediumTraverseRelease();
        m_climber.Stop(Arm.Both);
        barOneGrabbed = false;
      }else{
        m_climber.Start(-1, Arm.Both);
      }
    //Swing to third bar
    }else{
      if(m_climber.IsThirdBarLeftLimitSwitchPressed() || m_climber.IsThirdBarRightLimitSwitchPressed()){
        m_climber.MediumTraverseGrab();
        m_climber.Stop(Arm.Both);
        m_climber.EngageBrake();
        m_climber.HighRelease();
      }else{
        m_climber.Start(1, Arm.Both);
      }
    }
  }
}
