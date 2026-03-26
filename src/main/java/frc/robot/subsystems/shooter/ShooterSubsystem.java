package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

import java.util.Map;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX motor;

    private final VelocityVoltage velocityRequest =
        new VelocityVoltage(0).withSlot(0).withEnableFOC(false);

    private final Map<String, ShooterProfile> availableProfiles;
    private final SendableChooser<String> profileChooser;
    private ShooterProfile activeProfile;
    private String lastSelectedProfileName = "";

    private double  targetWheelRPM = 0.0;
    private boolean motorConfigured = false;

    private double  spinupStartTime = -1.0;
    private boolean isSpinningUp = false;
    public double currentRPM = 0;

    public ShooterSubsystem() {
        motor = new TalonFX(ShooterConstants.MOTOR_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        Slot0Configs slot0 = config.Slot0;
        slot0.kP = ShooterConstants.kP_TALON;
        slot0.kI = ShooterConstants.kI_TALON;
        slot0.kD = ShooterConstants.kD_TALON;
        slot0.kV = ShooterConstants.kV_TALON;
        slot0.kS = ShooterConstants.kS_TALON;

        motor.getConfigurator().apply(config);
        motorConfigured = true;

        availableProfiles = ShooterConstants.createAllProfiles();
        profileChooser    = new SendableChooser<>();

        boolean defaultSet = false;
        for (Map.Entry<String, ShooterProfile> entry : availableProfiles.entrySet()) {
            String profileName = entry.getKey();
            ShooterProfile profile = entry.getValue();
            if (profileName.equals(ShooterConstants.DEFAULT_PROFILE_NAME) && !defaultSet) {
                profileChooser.setDefaultOption(profile.getDisplayName(), profileName);
                defaultSet = true;
            } else {
                profileChooser.addOption(profile.getDisplayName(), profileName);
            }
        }

        SmartDashboard.putData("Shooter/Profile Selector", profileChooser);
        setActiveProfile(ShooterConstants.DEFAULT_PROFILE_NAME);
        SmartDashboard.putNumber("Shooter/Spinup Wait (s)", ShooterConstants.SPINUP_WAIT_SECONDS);
    }

    @Override
    public void periodic() {
        String selectedProfileName = profileChooser.getSelected();
        if (selectedProfileName != null && !selectedProfileName.equals(lastSelectedProfileName)) {
            setActiveProfile(selectedProfileName);
        }

        double waitTime = SmartDashboard.getNumber("Shooter/Spinup Wait (s)", ShooterConstants.SPINUP_WAIT_SECONDS);

        if (isSpinningUp) {
            double elapsed = Timer.getFPGATimestamp() - spinupStartTime;
            if (elapsed >= waitTime && atTargetVelocity()) {
                isSpinningUp = false;
            }
        }

        double elapsed = isSpinningUp ? (Timer.getFPGATimestamp() - spinupStartTime) : 0.0;

        SmartDashboard.putNumber("Shooter/Target Wheel RPM",targetWheelRPM);
        SmartDashboard.putNumber("Shooter/Target Motor RPM",targetWheelRPM * ShooterConstants.GEAR_RATIO);
        SmartDashboard.putNumber("Shooter/Actual Wheel RPM",getWheelRPM());
        SmartDashboard.putNumber("Shooter/Actual Motor RPM",getMotorRPM());
        SmartDashboard.putNumber("Shooter/RPM Error",targetWheelRPM - getWheelRPM());
        SmartDashboard.putBoolean("Shooter/At Target",atTargetVelocity());
        SmartDashboard.putBoolean("Shooter/Is Spinning Up",isSpinningUp);
        SmartDashboard.putNumber("Shooter/Spinup Elapsed (s)",elapsed);
        SmartDashboard.putNumber("Shooter/Spinup Remaining (s)",
            isSpinningUp ? Math.max(0, waitTime - elapsed) : 0.0);
        SmartDashboard.putBoolean("Shooter/Motor Configured",motorConfigured);

        if (activeProfile != null) {
            SmartDashboard.putString("Shooter/Active Profile",activeProfile.getName());
            SmartDashboard.putNumber("Shooter/Profile Angle (deg)",activeProfile.getAngleDegrees());
            SmartDashboard.putNumber("Shooter/Profile Min Dist (m)",activeProfile.getMinSafeDistance());
            SmartDashboard.putNumber("Shooter/Profile Max Dist (m)",activeProfile.getMaxSafeDistance());
        }
    }

    public void setVelocityForDistance(double distanceMeters) {
        if (activeProfile == null) {
            DriverStation.reportError("No active shooter profile", false);
            return;
        }
        double wheelRPM = getRPMForDistance(distanceMeters);
        setVelocityRPM(wheelRPM);
        SmartDashboard.putNumber("Shooter/Last Distance (m)", distanceMeters);
        SmartDashboard.putNumber("Shooter/Last Commanded RPM", wheelRPM);
    }

    public void updateVelocityForDistance(double distanceMeters) {
        if (activeProfile == null) return;
        double wheelRPM = getRPMForDistance(distanceMeters);

        if (Math.abs(wheelRPM - targetWheelRPM) < 50.0) return;

        targetWheelRPM = wheelRPM;
        double motorRPS = (wheelRPM * ShooterConstants.GEAR_RATIO) / 60.0;
        motor.setControl(velocityRequest.withVelocity(motorRPS));
        SmartDashboard.putNumber("Shooter/Last Distance (m)", distanceMeters);
        SmartDashboard.putNumber("Shooter/Last Commanded RPM", wheelRPM);
    }

    public void setVelocityRPM(double wheelRPM) {
        targetWheelRPM = wheelRPM;
        double motorRPS = (wheelRPM * ShooterConstants.GEAR_RATIO) / 60.0;
        motor.setControl(velocityRequest.withVelocity(motorRPS));
        spinupStartTime = Timer.getFPGATimestamp();
        isSpinningUp = true;
    }

    public void stop() {
        motor.stopMotor();
        targetWheelRPM  = 0.0;
        isSpinningUp = false;
        spinupStartTime = -1.0;
    }

    public double getWheelRPM() {
        return getMotorRPM() / ShooterConstants.GEAR_RATIO;
    }

    public double getMotorRPM() {
        return motor.getVelocity().getValueAsDouble() * 60.0;
    }

    public boolean atTargetVelocity() {
        if (targetWheelRPM == 0.0) return false;
        return Math.abs(getWheelRPM() - targetWheelRPM) < ShooterConstants.VELOCITY_TOLERANCE_RPM;
    }

    public boolean isReadyToShoot() {
        return atTargetVelocity() && !isSpinningUp;
    }
    public boolean isSpeedCriticallyLow() {
        if (targetWheelRPM == 0.0) return true;
        return (targetWheelRPM - getWheelRPM()) > ShooterConstants.VELOCITY_TOLERANCE_RPM;
    }

    public boolean isDistanceInRange(double distanceMeters) {
        if (activeProfile == null) return false;
        return activeProfile.isDistanceInRange(distanceMeters);
    }

    public String getActiveProfileName() {
        return activeProfile != null ? activeProfile.getName() : "NONE";
    }

    public double getActiveProfileAngle() {
        return activeProfile != null ? activeProfile.getAngleDegrees() : 0.0;
    }

    public void setActiveProfile(String profileName) {
        if (!availableProfiles.containsKey(profileName)) {
            DriverStation.reportError("Profile '" + profileName + "' not found. Using default.", false);
            profileName = ShooterConstants.DEFAULT_PROFILE_NAME;
        }
        activeProfile           = availableProfiles.get(profileName);
        lastSelectedProfileName = profileName;
        DriverStation.reportWarning(
            String.format("Shooter profile: %s (%.1f deg, %.1f-%.1fm)",
                activeProfile.getName(), activeProfile.getAngleDegrees(),
                activeProfile.getMinSafeDistance(), activeProfile.getMaxSafeDistance()),
            false);
    }

    private double getRPMForDistance(double distance) {
        if (activeProfile == null) {
            DriverStation.reportError("No active profile - using default RPM", false);
            return 3500.0;
        }
        if (distance < activeProfile.getMinSafeDistance()) {
            DriverStation.reportWarning(String.format(
                "Distance %.2fm below min %.2fm - clamping", distance, activeProfile.getMinSafeDistance()), false);
            SmartDashboard.putBoolean("Shooter/Distance In Range", false);
            return activeProfile.getRPMForDistance(activeProfile.getMinSafeDistance());
        }
        if (distance > activeProfile.getMaxSafeDistance()) {
            DriverStation.reportWarning(String.format(
                "Distance %.2fm exceeds max %.2fm - clamping", distance, activeProfile.getMaxSafeDistance()), false);
            SmartDashboard.putBoolean("Shooter/Distance In Range", false);
            return activeProfile.getRPMForDistance(activeProfile.getMaxSafeDistance());
        }
        SmartDashboard.putBoolean("Shooter/Distance In Range", true);
        return activeProfile.getRPMForDistance(distance);
    }
}