package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

import java.util.function.DoubleSupplier;

/**
 * ContinuousAimCommand
 *
 * Continuously rotates the robot to face the hub while allowing full
 * translational driving. Does NOT spin up the shooter or fire.
 * Bind this to a held button; pair with HubShootCommand on a second button.
 *
 * AdvantageScope telemetry is published under "ContinuousAim/" via NT4.
 */
public class ContinuousAimCommand extends Command {

    // ── Hub positions (field-relative, WPILib Blue origin) ──────────────────
    private static final Translation2d BLUE_HUB = new Translation2d(4.552, 4.021);
    private static final Translation2d RED_HUB  = new Translation2d(11.961, 4.021);

    // ── Rotation P-controller ────────────────────────────────────────────────
    /** rad/s output per degree of error */
    private static final double KP_ROTATION            = 0.065;
    /** Minimum output to overcome static friction (rad/s) */
    private static final double MIN_COMMAND_RADPS      = 0.03;
    /** Error band where min-command boost is suppressed */
    private static final double ERROR_DEADZONE_DEG     = 1.5;
    /** Error band considered "aligned" */
    private static final double ALIGNED_TOLERANCE_DEG  = 2.5;
    /** Max rotation rate this command will ever command */
    private static final double MAX_ROT_RADPS          = Units.rotationsToRadians(2.0);

    // ── Drive input ──────────────────────────────────────────────────────────
    private static final double JOYSTICK_DEADBAND = 0.10;

    // ── Dependencies ────────────────────────────────────────────────────────
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier leftYSupplier;
    private final DoubleSupplier leftXSupplier;
    private final double maxSpeed;

    // ── State ────────────────────────────────────────────────────────────────
    private boolean isAligned = false;

    // ── NT4 publishers (AdvantageScope) ──────────────────────────────────────
    private final NetworkTable          nt;
    private final DoublePublisher       ntRobotX;
    private final DoublePublisher       ntRobotY;
    private final DoublePublisher       ntRobotHeadingDeg;
    private final DoublePublisher       ntTargetHeadingDeg;
    private final DoublePublisher       ntRotErrorDeg;
    private final DoublePublisher       ntRotOutputRadps;
    private final DoublePublisher       ntDistanceToHubM;
    private final BooleanPublisher      ntAligned;
    private final BooleanPublisher      ntRunning;
    private final StringPublisher       ntAlliance;

    // ── Swerve request (reused every loop to avoid GC pressure) ─────────────
    private final SwerveRequest.FieldCentric driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public ContinuousAimCommand(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier leftYSupplier,
            DoubleSupplier leftXSupplier,
            double maxSpeed) {

        this.drivetrain    = drivetrain;
        this.leftYSupplier = leftYSupplier;
        this.leftXSupplier = leftXSupplier;
        this.maxSpeed      = maxSpeed;

        addRequirements(drivetrain);

        // Open NT4 table once — publishers are reused every loop
        nt = NetworkTableInstance.getDefault().getTable("ContinuousAim");

        ntRobotX           = nt.getDoubleTopic("Robot_X_m").publish();
        ntRobotY           = nt.getDoubleTopic("Robot_Y_m").publish();
        ntRobotHeadingDeg  = nt.getDoubleTopic("Robot_Heading_deg").publish();
        ntTargetHeadingDeg = nt.getDoubleTopic("Target_Heading_deg").publish();
        ntRotErrorDeg      = nt.getDoubleTopic("Rotation_Error_deg").publish();
        ntRotOutputRadps   = nt.getDoubleTopic("Rotation_Output_radps").publish();
        ntDistanceToHubM   = nt.getDoubleTopic("Distance_To_Hub_m").publish();
        ntAligned          = nt.getBooleanTopic("Aligned").publish();
        ntRunning          = nt.getBooleanTopic("Running").publish();
        ntAlliance         = nt.getStringTopic("Alliance").publish();
    }

    // ── Lifecycle ────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        isAligned = false;
        ntRunning.set(true);
        ntAligned.set(false);
    }

    @Override
    public void execute() {
        // 1. Read joystick with deadband
        double rawY = leftYSupplier.getAsDouble();
        double rawX = leftXSupplier.getAsDouble();
        double forwardMps = applyDeadband(rawY) * -maxSpeed;
        double strafeMps  = applyDeadband(rawX) * -maxSpeed;

        // 3. Geometry
        Pose2d robotPose = drivetrain.getState().Pose;
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d hub = (alliance == Alliance.Blue) ? BLUE_HUB : RED_HUB;

        Translation2d toHub     = hub.minus(robotPose.getTranslation());
        double        distance  = toHub.getNorm();
        Rotation2d    targetHdg = new Rotation2d(toHub.getX(), toHub.getY());

        double errorDeg = wrapDegrees(targetHdg.getDegrees() - robotPose.getRotation().getDegrees());

        // 4. P-controller with static-friction boost
        double output;
        if (Math.abs(errorDeg) > ERROR_DEADZONE_DEG) {
            output  = KP_ROTATION * errorDeg;
            output += (errorDeg > 0) ? MIN_COMMAND_RADPS : -MIN_COMMAND_RADPS;
        } else {
            output = KP_ROTATION * errorDeg;
        }
        output = Math.max(-MAX_ROT_RADPS, Math.min(MAX_ROT_RADPS, output));

        isAligned = Math.abs(errorDeg) <= ALIGNED_TOLERANCE_DEG;

        // 5. Apply drive request
        drivetrain.setControl(
                driveRequest
                        .withVelocityX(forwardMps)
                        .withVelocityY(strafeMps)
                        .withRotationalRate(output));

        // 6. Publish AdvantageScope telemetry
        ntRobotX.set(robotPose.getX());
        ntRobotY.set(robotPose.getY());
        ntRobotHeadingDeg.set(robotPose.getRotation().getDegrees());
        ntTargetHeadingDeg.set(targetHdg.getDegrees());
        ntRotErrorDeg.set(errorDeg);
        ntRotOutputRadps.set(output);
        ntDistanceToHubM.set(distance);
        ntAligned.set(isAligned);
        ntAlliance.set(alliance.toString());
    }

    @Override
    public void end(boolean interrupted) {
        ntRunning.set(false);
        ntAligned.set(false);
    }

    /** This command runs indefinitely — cancel it by releasing the button. */
    @Override
    public boolean isFinished() {
        return false;
    }

    // ── Public query methods (used by HubShootCommand) ───────────────────────

    public boolean isAligned() {
        return isAligned;
    }

    public double getDistanceToHub() {
        Pose2d robotPose = drivetrain.getState().Pose;
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d hub = (alliance == Alliance.Blue) ? BLUE_HUB : RED_HUB;
        return hub.minus(robotPose.getTranslation()).getNorm();
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private static double applyDeadband(double value) {
        return Math.abs(value) > JOYSTICK_DEADBAND ? value : 0.0;
    }

    /** Wraps an angle to (-180, 180]. */
    private static double wrapDegrees(double deg) {
        double mod = deg % 360.0;
        if (mod >  180.0) mod -= 360.0;
        if (mod <= -180.0) mod += 360.0;
        return mod;
    }
}