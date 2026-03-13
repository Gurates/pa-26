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
 * ShootCommand
 *
 * While held:
 *   1. Spins up shooter using live odometry distance to hub
 *   2. Once isReadyToShoot() → runs hopper + feeder
 *   3. If ball slows the wheel → pauses feeding, spins back up, then resumes
 *
 * On release / interrupt → everything stops cleanly.
 */
public class ShootCommand extends Command {

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

    public ShootCommand(
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
        double distance = getDistanceToHub();
        shooter.setVelocityForDistance(distance);
        lastCommandedDistance = distance;
        SmartDashboard.putString("ShootCommand/Phase", phase.toString());
        SmartDashboard.putNumber("ShootCommand/Distance (m)", distance);
        SmartDashboard.putNumber("RPM", shooter.getMotorRPM());
    }

    @Override
    public void execute() {
        double distance = getDistanceToHub();
        SmartDashboard.putNumber("ShootCommand/Distance (m)", distance);
        SmartDashboard.putBoolean("ShootCommand/Shooter Ready", shooter.isReadyToShoot());

        // Only re-command shooter if distance changed meaningfully (avoids resetting spinup timer)
        boolean distanceChanged = Math.abs(distance - lastCommandedDistance) > 0.05;

        switch (phase) {

            case SPINNING_UP:
                if (distanceChanged) {
                    shooter.setVelocityForDistance(distance);
                    lastCommandedDistance = distance;
                }

                if (shooter.isReadyToShoot()) {
                    phase = Phase.FEEDING;
                    SmartDashboard.putString("ShootCommand/Phase", phase.toString());
                }
                break;

            case FEEDING:
                if (distanceChanged) {
                    shooter.setVelocityForDistance(distance);
                    lastCommandedDistance = distance;
                }
                hopper.feed();
                feeder.feed();

                // If ball load drops wheel speed, pause and spin back up
                if (!shooter.isReadyToShoot()) {
                    phase = Phase.SPINNING_UP;
                    hopper.stop();
                    feeder.stop();
                    SmartDashboard.putString("ShootCommand/Phase", phase.toString());
                }
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        hopper.stop();
        feeder.stop();
        SmartDashboard.putString("ShootCommand/Phase", "IDLE");
        SmartDashboard.putBoolean("ShootCommand/Shooter Ready", false);
    }

    /** Runs forever — cancelled by releasing the button. */
    @Override
    public boolean isFinished() {
        return false;
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private double getDistanceToHub() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d hub = (alliance == Alliance.Blue) ? BLUE_HUB : RED_HUB;
        return hub.minus(drivetrain.getState().Pose.getTranslation()).getNorm();
    }
}