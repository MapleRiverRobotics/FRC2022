package frc.robot.commands;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TrajectoryHelper;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoFourBallUpperHub  extends SequentialCommandGroup  {

    public AutoFourBallUpperHub(Drivetrain drivetrain, Shooter shooter, Indexer indexer, Intake intake) {

        Trajectory t1 = TrajectoryHelper.loadTrajectoryFromFile("PickUpFirstBall");
        Trajectory t2 = TrajectoryHelper.loadTrajectoryFromFile("DriveToTerminal");
        Trajectory t3 = TrajectoryHelper.loadTrajectoryFromFile("MoveUpToShoot");

        addCommands(
            new InstantCommand(() -> {
                drivetrain.resetOdometry(t1.getInitialPose());
              }),
              new IntakeLift(intake, 0).withTimeout(.5),
              new ParallelDeadlineGroup(
                TrajectoryHelper.createCommandForTrajectory(t1, true, drivetrain).withTimeout(5).withName("Forward"),
                new IntakeRun(intake, 1, .80)
              ),
              new DriveRotateInPlace(drivetrain, 180).withTimeout(4).withName("Turn180"),
              new Shoot(shooter, indexer, 2500).withTimeout(4),
              new DriveRotateInPlace(drivetrain, 180).withTimeout(4).withName("Turn180"),
              new ParallelCommandGroup(
                TrajectoryHelper.createCommandForTrajectory(t2, true, drivetrain).withTimeout(5).withName("Forward"),
                new IntakeRun(intake, 1, .80).withTimeout(5)
              ),

              new DriveRotateInPlace(drivetrain, 180).withTimeout(4).withName("Turn180"),
              new ParallelCommandGroup(
                new IntakeLift(intake, 0).withTimeout(1),
                TrajectoryHelper.createCommandForTrajectory(t3, true, drivetrain).withTimeout(5).withName("Forward")
              ),
              new AimAndShoot(shooter, drivetrain, indexer).withTimeout(6)
        );
    }
}
