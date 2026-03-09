package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import frc.robot.constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax intakeMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;

    private double targetPosition = IntakeConstants.INTAKE_RETRACTED_DEGREES;
    private boolean manualMode = false;

    private boolean holdingPosition = false;

    public IntakeSubsystem() {
        intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake)
              .smartCurrentLimit(30)
              .inverted(false);

        double positionConversionFactor = 360.0 / IntakeConstants.GEAR_RATIO;
        config.encoder
              .positionConversionFactor(positionConversionFactor)
              .velocityConversionFactor(positionConversionFactor / 60.0);

        config.closedLoop
              .pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD)
              .positionWrappingEnabled(false);

        config.softLimit
              .forwardSoftLimit(IntakeConstants.INTAKE_EXTENDED_DEGREES + 10)
              .forwardSoftLimitEnabled(true)
              .reverseSoftLimit(IntakeConstants.INTAKE_RETRACTED_DEGREES - 10)
              .reverseSoftLimitEnabled(true);

        config.signals
              .outputCurrentPeriodMs(20)
              .appliedOutputPeriodMs(20)
              .busVoltagePeriodMs(20);

        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.clearFaults();

        encoder = intakeMotor.getEncoder();
        pidController = intakeMotor.getClosedLoopController();

        encoder.setPosition(IntakeConstants.INTAKE_RETRACTED_DEGREES);
    }

    public void extend() {
        setTargetPosition(IntakeConstants.INTAKE_EXTENDED_DEGREES);
    }

    public void retract() {
        setTargetPosition(IntakeConstants.INTAKE_RETRACTED_DEGREES);
    }

    public void toggle() {
        if (isRetracted()) {
            extend();
        } else {
            retract();
        }
    }

    public void setSpeed(double speed) {
        manualMode = true;
        holdingPosition = false;
        intakeMotor.set(speed);
        targetPosition = encoder.getPosition();
    }

    public void stop() {
        intakeMotor.stopMotor();
        targetPosition = encoder.getPosition();
        holdingPosition = false;
        manualMode = false;
    }

    public void calibrate() {
        encoder.setPosition(IntakeConstants.INTAKE_RETRACTED_DEGREES);
        targetPosition = IntakeConstants.INTAKE_RETRACTED_DEGREES;
        manualMode = false;
        holdingPosition = false;
        System.out.println("Intake encoder calibrated to " + IntakeConstants.INTAKE_RETRACTED_DEGREES + " degrees");
    }

    public boolean isExtended() {
        return Math.abs(encoder.getPosition() - IntakeConstants.INTAKE_EXTENDED_DEGREES)
               < IntakeConstants.POSITION_TOLERANCE;
    }

    public boolean isRetracted() {
        return Math.abs(encoder.getPosition() - IntakeConstants.INTAKE_RETRACTED_DEGREES)
               < IntakeConstants.POSITION_TOLERANCE;
    }

    public boolean atTarget() {
        boolean positionOk = Math.abs(encoder.getPosition() - targetPosition)
                             < IntakeConstants.POSITION_TOLERANCE;
        boolean velocityOk = Math.abs(getVelocity()) < 5.0; // deg/s
        return positionOk && velocityOk;
    }

    public double getPosition()      { return encoder.getPosition(); }
    public double getVelocity()      { return encoder.getVelocity(); }
    public double getTargetPosition(){ return targetPosition; }
    public double getMotorCurrent()  { return intakeMotor.getOutputCurrent(); }
    public boolean isManualMode()    { return manualMode; }

    @Deprecated
    public void resetEncoder() { calibrate(); }

    private void setTargetPosition(double positionDeg) {
        holdingPosition = false;
        manualMode = false;
        targetPosition = positionDeg;
        pidController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void periodic() {
        // Reached target → stop motor, brake mode holds — eliminates oscillation
        if (!manualMode && !holdingPosition && atTarget()) {
            intakeMotor.stopMotor();
            holdingPosition = true;
        }

        // Drifted out of tolerance → re-engage PID to correct
        if (!manualMode && holdingPosition && !atTarget()) {
            holdingPosition = false;
            pidController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        SmartDashboard.putNumber("Intake/Position (deg)", getPosition());
        SmartDashboard.putNumber("Intake/Target Position (deg)", targetPosition);
        SmartDashboard.putNumber("Intake/Velocity (deg/s)", getVelocity());
        SmartDashboard.putNumber("Intake/Position Error", targetPosition - getPosition());
        SmartDashboard.putBoolean("Intake/Is Extended", isExtended());
        SmartDashboard.putBoolean("Intake/Is Retracted", isRetracted());
        SmartDashboard.putBoolean("Intake/At Target", atTarget());
        SmartDashboard.putBoolean("Intake/Holding Position", holdingPosition);
        SmartDashboard.putNumber("Intake/Motor Current", getMotorCurrent());
        SmartDashboard.putNumber("Intake/Applied Output", intakeMotor.getAppliedOutput());
        SmartDashboard.putNumber("Intake/Bus Voltage", intakeMotor.getBusVoltage());
        SmartDashboard.putBoolean("Intake/Manual Mode", manualMode);
        SmartDashboard.putBoolean("Intake/High Current Warning", getMotorCurrent() > 25.0);

        boolean isStalled = !manualMode && !holdingPosition &&
                            Math.abs(getVelocity()) < 0.5 && !atTarget() &&
                            Math.abs(targetPosition - getPosition()) > IntakeConstants.POSITION_TOLERANCE;
        SmartDashboard.putBoolean("Intake/Stalled", isStalled);

        boolean outOfBounds = getPosition() < (IntakeConstants.INTAKE_RETRACTED_DEGREES - 15) ||
                              getPosition() > (IntakeConstants.INTAKE_EXTENDED_DEGREES + 15);
        SmartDashboard.putBoolean("Intake/Out of Bounds", outOfBounds);
    }
}