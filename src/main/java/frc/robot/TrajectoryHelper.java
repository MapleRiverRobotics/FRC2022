package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.Drivetrain;

public class TrajectoryHelper {
  
  public static final DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(
    TrajectoryConstants.kTrackWidthMeters);


    public static Trajectory loadTrajectoryFromFile(String pathName) {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/" + pathName + ".wpilib.json");
            return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
          } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + pathName, ex.getStackTrace());
          }
          return new Trajectory();
      }

        /**
   * Creates a command to follow a Trajectory on the drivetrain.
   * @param trajectory trajectory to follow
   * @return command that will run the trajectory
   */
  public static Command createCommandForTrajectory(Trajectory trajectory, Boolean initPose, Drivetrain drivetrain) {
    
    drivetrain.resetEncoders();

    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        drivetrain::getPose,
        new RamseteController(TrajectoryConstants.kRamseteB, TrajectoryConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            TrajectoryConstants.ksVolts,
            TrajectoryConstants.kvVoltSecondsPerMeter,
            TrajectoryConstants.kaVoltSecondsSquaredPerMeter),
        m_driveKinematics,
        drivetrain::getWheelSpeeds,
        new PIDController(TrajectoryConstants.kPDriveVel, 0, 0),
        new PIDController(TrajectoryConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        drivetrain::tankDriveVolts,
        drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    //return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));

    if (initPose) {
      var reset =  new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialPose()));
      return reset.andThen(ramseteCommand.andThen(() -> drivetrain.stop()));
    }
    else {
      return ramseteCommand.andThen(() -> drivetrain.stop());
    }
  }

  private static double inchesToMeters(double inches) {
    return inches * .0254;
  }

  public static Command getDriveStraightCommand(Drivetrain drivetrain, double inches) {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            TrajectoryConstants.ksVolts,
            TrajectoryConstants.kvVoltSecondsPerMeter,
            TrajectoryConstants.kaVoltSecondsSquaredPerMeter),
        m_driveKinematics,
        10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        TrajectoryConstants.kMaxSpeedMetersPerSecond,
        TrajectoryConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(m_driveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint)
            .setReversed(inches < 0);

    double startLocation = 0;
    double endLocation = 0;
    if (inches < 0) {
      startLocation = inchesToMeters(inches) * -1;
    } else {
      endLocation = inchesToMeters(inches);
    }

    // An example trajectory to follow. All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(startLocation, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
   //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
         List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(endLocation, 0, new Rotation2d(0)),
        // Pass config
        config);

    return TrajectoryHelper.createCommandForTrajectory(trajectory, true, drivetrain);
  }

  

}
