// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ClimberConstants.Arm;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();
  public static OI oi = new OI();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Shooter m_shooter = new Shooter();
  private final Climber m_climber = new Climber();
  private final Intake m_intake = new Intake();
  private final Indexer m_indexer = new Indexer();

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Joysticks
  private final Joystick driveJoystick = new Joystick(OIConstants.DriverJoystickId);
  private final XboxController operatorJoystick = new XboxController(OIConstants.OperatorJoystickId);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {

    // Smartdashboard Subsystems
    SmartDashboard.putData(m_drivetrain);

    // SmartDashboard Buttons
    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Auto Two Ball Upper Hub", new AutoTwoBallUpperHub(m_drivetrain, m_shooter, m_indexer, m_intake));
    m_chooser.addOption("Auto Two Ball Lower Hub", new AutoTwoBallLowerHub(m_drivetrain, m_shooter, m_indexer, m_intake));
    m_chooser.addOption("Auto Four Ball", new AutoFourBallUpperHub(m_drivetrain, m_shooter, m_indexer, m_intake));

    // Put the chooser on the dashboard
    SmartDashboard.putData(m_chooser);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_drivetrain.setDefaultCommand(new Drive(m_drivetrain));
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Create Shooter buttons
    final JoystickButton shooterShootButtonOne = new JoystickButton(driveJoystick, 1);
    shooterShootButtonOne.whileTrue(new Shoot(m_shooter, m_indexer, 4000));
    // final JoystickButton shooterShootButtonTwo = new JoystickButton(driveJoystick, 12);
    // shooterShootButtonTwo.whileActiveContinuous(new Shoot(m_shooter, m_indexer, 2000));
    // final JoystickButton shooterShootButtonThree = new JoystickButton(driveJoystick, 11);
    // shooterShootButtonThree.whileActiveContinuous(new Shoot(m_shooter, m_indexer, 2500));

    final JoystickButton shooterAimAndShoot = new JoystickButton(driveJoystick, 2);
    shooterAimAndShoot.whileTrue(new AimAndShoot(m_shooter, m_drivetrain, m_indexer));

    // Intake Triggers
    Trigger intakeForward = new Trigger(() -> operatorJoystick.getPOV() > 315 ||
        (operatorJoystick.getPOV() >= 0 && operatorJoystick.getPOV() < 45));


    Trigger intakeReverse = new Trigger(() -> operatorJoystick.getPOV() > 135 && operatorJoystick.getPOV() < 215);

    
    intakeForward.whileTrue(new IntakeRun(m_intake, -1, .80));


    intakeReverse.whileTrue(new IntakeRun(m_intake, 1, .80));


    final JoystickButton intakePowerBoostButton = new JoystickButton(operatorJoystick, IntakeConstants.IntakePowerBoost);
    intakePowerBoostButton.whileTrue(new IntakeRun(m_intake, 1, 100));

    final JoystickButton intakeUpButton = new JoystickButton(operatorJoystick, IntakeConstants.IntakeUpButton);
    intakeUpButton.onTrue(new IntakeLift(m_intake, 1));
    final JoystickButton intakeDownButton = new JoystickButton(operatorJoystick, IntakeConstants.IntakeDownButton);
    intakeDownButton.onTrue(new IntakeLift(m_intake, 0));

    // Indexer Triggers
    Trigger indexerForward = new Trigger(() -> operatorJoystick.getPOV() >= 45 && operatorJoystick.getPOV() < 135);
    Trigger indexerReverse = new Trigger(() -> operatorJoystick.getPOV() > 225 && operatorJoystick.getPOV() < 315);
    indexerForward.whileTrue(new IndexRun(m_indexer, 1));
    indexerReverse.whileTrue(new IndexRun(m_indexer, -1));

    // Climber Rotation Triggers
    Trigger leftTrigger = new Trigger(() -> operatorJoystick.getLeftTriggerAxis() > 0);
    Trigger rightTrigger = new Trigger(() -> operatorJoystick.getRightTriggerAxis() > 0);
    Trigger leftBumper = new Trigger(() -> operatorJoystick.getLeftBumper());
    Trigger rightBumper = new Trigger(() -> operatorJoystick.getRightBumper());

    leftTrigger.and(rightTrigger).whileTrue(new Rotate(m_climber, Arm.Both, 1));
    leftTrigger.negate().and(rightTrigger).whileTrue(new Rotate(m_climber, Arm.Right, 1));
    rightTrigger.negate().and(leftTrigger).whileTrue(new Rotate(m_climber, Arm.Left, 1));

    leftBumper.and(rightBumper).whileTrue(new Rotate(m_climber, Arm.Both, -1));
    leftBumper.negate().and(rightBumper).whileTrue(new Rotate(m_climber, Arm.Right, -1));
    rightBumper.negate().and(leftBumper).whileTrue(new Rotate(m_climber, Arm.Left, -1));

    // Climber Pneumatic Buttons
    final JoystickButton releaseButton = new JoystickButton(operatorJoystick, 9);
    final JoystickButton grab1Button = new JoystickButton(operatorJoystick, 3);
    final JoystickButton grab2Button = new JoystickButton(operatorJoystick, 4);
    //final JoystickButton grab3Button = new JoystickButton(operatorJoystick, 2);
    final JoystickButton release3Buton = new JoystickButton(operatorJoystick, 1);

    // Grab
    // releaseButton.negate().and(grab1Button).whileActiveOnce(new Grab(m_climber,
    // 1));
    // releaseButton.negate().and(grab2Button).whileActiveOnce(new Grab(m_climber,
    // 2));
    // releaseButton.negate().and(grab3Button).whileActiveOnce(new Grab(m_climber,
    // 3));
    // releaseButton.negate().and(grab1Button).whileActiveContinuous(new AutoClimb(m_climber, 1));
    // releaseButton.negate().and(grab2Button).whileActiveContinuous(new AutoClimb(m_climber, 2));
    // releaseButton.negate().and(grab3Button).whileActiveContinuous(new AutoClimb(m_climber, 3));
    // releaseButton.negate().and(release3Buton).whileActiveContinuous(new AutoClimb(m_climber, 4));

    // auto climb with driver joystick
    final JoystickButton driverBarOne = new JoystickButton(driveJoystick, 5);
    driverBarOne.whileTrue(new AutoClimb(m_climber, 1));
    final JoystickButton driverBarTwo = new JoystickButton(driveJoystick, 6);
    driverBarTwo.whileTrue(new AutoClimb(m_climber, 2));
    final JoystickButton driverBarThree = new JoystickButton(driveJoystick, 3);
    driverBarThree.whileTrue(new AutoClimb(m_climber, 3));
    final JoystickButton driverBarFour = new JoystickButton(driveJoystick, 4);
    driverBarFour.whileTrue(new AutoClimb(m_climber, 4));


    //Manual Lift Driver joystick
    //Grab
    final JoystickButton grabDrive1Button = new JoystickButton(driveJoystick, 11);
    grabDrive1Button.onTrue(new Grab(m_climber, 1));
    final JoystickButton grabDrive2Button = new JoystickButton(driveJoystick, 9);
    grabDrive2Button.onTrue(new Grab(m_climber, 2));
    final JoystickButton grabDrive3Buton = new JoystickButton(driveJoystick, 7);
    grabDrive3Buton.onTrue(new Grab(m_climber, 3));

    //Release
    final JoystickButton releaseDrive1Button = new JoystickButton(driveJoystick, 12);
    releaseDrive1Button.onTrue(new Release(m_climber, 1));
    final JoystickButton releaseDrive2Button = new JoystickButton(driveJoystick, 10);
    releaseDrive2Button.onTrue(new Release(m_climber, 2));
    final JoystickButton releaseDrive3Buton = new JoystickButton(driveJoystick, 8);
    releaseDrive3Buton.onTrue(new Release(m_climber, 3));

    //Rotate
    Trigger driveLiftForward = new Trigger(() -> driveJoystick.getPOV() > 315 ||
    (driveJoystick.getPOV() >= 0 && driveJoystick.getPOV() < 45));
    Trigger driveLiftReverse = new Trigger(() -> driveJoystick.getPOV() > 135 && operatorJoystick.getPOV() < 215);
    driveLiftForward.whileTrue(new Rotate(m_climber, Arm.Both, -1));
    driveLiftReverse.whileTrue(new Rotate(m_climber, Arm.Both, 1));

    // final JoystickButton driverSwitchStatus = new JoystickButton(driveJoystick, 7);
    // driverSwitchStatus.whileActiveContinuous(new AutoClimb(m_climber, 5));

    // Release
    releaseButton.and(grab1Button).whileTrue(new Release(m_climber, 1));
    releaseButton.and(grab2Button).whileTrue(new Release(m_climber, 2));
   // releaseButton.and(grab3Button).whileActiveOnce(new Release(m_climber, 3));

  //  NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  //  limelight.getEntry("ledMode").setNumber(3);
  }

  public Joystick getDriveJoystick() {
    return driveJoystick;
  }

  /**
  * Use this to pass the autonomous command to the main {@link Robot} class.
  *
  * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
  // The selected command will be run in autonomous
  return m_chooser.getSelected();
  }

}
