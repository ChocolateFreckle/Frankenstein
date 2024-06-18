
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.generated.TunerConstants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs(){
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = TunerConstants.Swerve.cancoderInvert;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = TunerConstants.Swerve.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = TunerConstants.Swerve.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = TunerConstants.Swerve.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = TunerConstants.Swerve.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = TunerConstants.Swerve.angleCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = TunerConstants.Swerve.angleCurrentThreshold;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = TunerConstants.Swerve.angleCurrentThresholdTime;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = TunerConstants.steerGains.kP;
        swerveAngleFXConfig.Slot0.kI = TunerConstants.steerGains.kI;
        swerveAngleFXConfig.Slot0.kD = TunerConstants.steerGains.kD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = TunerConstants.Swerve.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = TunerConstants.Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = TunerConstants.Swerve.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = TunerConstants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = TunerConstants.Swerve.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = TunerConstants.Swerve.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = TunerConstants.Swerve.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = TunerConstants.driveGains.kP;
        swerveDriveFXConfig.Slot0.kI = TunerConstants.driveGains.kI;
        swerveDriveFXConfig.Slot0.kD = TunerConstants.driveGains.kD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = TunerConstants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = TunerConstants.Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = TunerConstants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = TunerConstants.Swerve.closedLoopRamp;
    }
}
