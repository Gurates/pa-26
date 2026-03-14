package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.ContinuousAimCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hooper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {

    private double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem();
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final HopperSubsystem hopper = new HopperSubsystem();
    public final FeederSubsystem feeder = new FeederSubsystem();

    private final SendableChooser<Command> autoChooser;

    private ContinuousAimCommand continuousAim;

    public RobotContainer() {
        NamedCommands.registerCommand("AutoShoot",
                new AutoShootCommand(shooter, hopper, feeder, drivetrain));

        autoChooser = AutoBuilder.buildAutoChooser("Tests");

        configureBindings();

        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {

        // ── Default drive ─────────────────────────────────────────────────────
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // ── Drive utility ─────────────────────────────────────────────────────
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // ── SysId ─────────────────────────────────────────────────────────────
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // ── Continuous Aim — hold left trigger ────────────────────────────────
        continuousAim = new ContinuousAimCommand(
                drivetrain,
                () -> joystick.getLeftY(),
                () -> joystick.getLeftX(),
                MaxSpeed);

        joystick.leftTrigger().whileTrue(continuousAim);

        // ── Shoot — hold right bumper ──────────────────────────────────────────
        // Spins up shooter to distance-based RPM, feeds when ready.
        // Pair with left trigger (ContinuousAim) to aim and shoot simultaneously.
        joystick.rightBumper().whileTrue(
                new ShootCommand(shooter, hopper, feeder, drivetrain));

        // ── Intake extend/retract — Y and X buttons ───────────────────────────
        joystick.y().onTrue(Commands.runOnce(intake::extend, intake));
        joystick.x().onTrue(Commands.runOnce(intake::retract, intake));

        // ── Intake roller — right trigger ─────────────────────────────────────
        joystick.rightTrigger().whileTrue(
                Commands.startEnd(
                        intakeRoller::intake,
                        intakeRoller::stop,
                        intakeRoller));

        drivetrain.registerTelemetry(logger::telemeterize);

        SmartDashboard.putData("Autonomous/Auto Chooser", autoChooser);
    }
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}