package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hooper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * AutoShoot15Command
 *
 * Spins up the shooter using live odometry distance, then runs
 * hopper + feeder continuously for 15 seconds.
 *
 * If the wheel drops below target during feeding, feeding pauses
 * and resumes once the shooter is back up to speed.
 *
 * Register in RobotContainer:
 *   NamedCommands.registerCommand("Shoot15",
 *       new AutoShoot15Command(shooter, hopper, feeder, drivetrain));
 */
public class AutoShootCommand extends Command {

    private static final double SHOOT_DURATION_SECONDS = 15.0;

    // ── Hub positions (WPILib Blue origin) ───────────────────────────────────
    private static final Translation2d BLUE_HUB = new Translation2d(4.552, 4.021);
    private static final Translation2d RED_HUB  = new Translation2d(11.961, 4.021);

    // ── Subsystems ───────────────────────────────────────────────────────────
    private final ShooterSubsystem        shooter;
    private final HopperSubsystem         hopper;
    private final FeederSubsystem         feeder;
    private final CommandSwerveDrivetrain drivetrain;

    // ── State ────────────────────────────────────────────────────────────────
    private enum Phase { SPINNING_UP, FEEDING }
    private Phase phase;
    private double lastCommandedDistance = -1.0;
    private double startTime = -1.0;

    public AutoShootCommand(
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            FeederSubsystem feeder,
            CommandSwerveDrivetrain drivetrain) {

        this.shooter    = shooter;
        this.hopper     = hopper;
        this.feeder     = feeder;
        this.drivetrain = drivetrain;

        addRequirements(shooter, hopper, feeder);
    }

    // ── Lifecycle ────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        phase = Phase.SPINNING_UP;
        lastCommandedDistance = -1.0;
        startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        double distance = getDistanceToHub();
        shooter.setVelocityForDistance(distance);
        lastCommandedDistance = distance;

        System.out.printf("[AutoShoot15] Started — %.2fm to hub%n", distance);
        SmartDashboard.putString("AutoShoot15/Phase", phase.toString());
        SmartDashboard.putNumber("AutoShoot15/Distance (m)", distance);
    }

    @Override
    public void execute() {
        double distance = getDistanceToHub();
        double elapsed  = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime;
        boolean distanceChanged = Math.abs(distance - lastCommandedDistance) > 0.05;

        SmartDashboard.putNumber("AutoShoot15/Distance (m)", distance);
        SmartDashboard.putNumber("AutoShoot15/Elapsed (s)", elapsed);
        SmartDashboard.putNumber("AutoShoot15/Remaining (s)", Math.max(0, SHOOT_DURATION_SECONDS - elapsed));
        SmartDashboard.putBoolean("AutoShoot15/Shooter Ready", shooter.isReadyToShoot());
        SmartDashboard.putString("AutoShoot15/Phase", phase.toString());

        switch (phase) {

            case SPINNING_UP:
                if (distanceChanged) {
                    shooter.setVelocityForDistance(distance);
                    lastCommandedDistance = distance;
                }
                if (shooter.isReadyToShoot()) {
                    phase = Phase.FEEDING;
                    System.out.println("[AutoShoot15] At speed — feeding");
                }
                break;

            case FEEDING:
                if (distanceChanged) {
                    shooter.setVelocityForDistance(distance);
                    lastCommandedDistance = distance;
                }
                hopper.feed();
                feeder.feed();

                // If a ball slows the wheel, pause feeding and spin back up
                if (!shooter.isReadyToShoot()) {
                    phase = Phase.SPINNING_UP;
                    hopper.stop();
                    feeder.stop();
                }
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        hopper.stop();
        feeder.stop();
        SmartDashboard.putString("AutoShoot15/Phase", "IDLE");
        System.out.println("[AutoShoot15] " + (interrupted ? "Interrupted" : "Complete"));
    }

    @Override
    public boolean isFinished() {
        double elapsed = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime;
        return elapsed >= SHOOT_DURATION_SECONDS;
    }

    // ── Helper ───────────────────────────────────────────────────────────────

    private double getDistanceToHub() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d hub = (alliance == Alliance.Blue) ? BLUE_HUB : RED_HUB;
        return hub.minus(drivetrain.getState().Pose.getTranslation()).getNorm();
    }
}