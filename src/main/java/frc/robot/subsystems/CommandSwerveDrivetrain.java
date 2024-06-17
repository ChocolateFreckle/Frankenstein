package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;//this was me
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {


    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    public SwerveDriveOdometry odometry;

    public final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
        TunerConstants.kFrontLeftDriveMotorId,
        TunerConstants.kFrontLeftSteerMotorId,
        TunerConstants.kFrontLeftEncoderOffset);
  
    public final MAXSwerveModule m_frontRight = new MAXSwerveModule(
        TunerConstants.kFrontRightDriveMotorId,
        TunerConstants.kFrontRightSteerMotorId,
        TunerConstants.kFrontRightEncoderOffset);
  
    public final MAXSwerveModule m_backLeft = new MAXSwerveModule(
        TunerConstants.kBackLeftDriveMotorId,
        TunerConstants.kBackLeftSteerMotorId,
        TunerConstants.kBackLeftEncoderOffset);
  
    public final MAXSwerveModule m_backRight = new MAXSwerveModule(
        TunerConstants.kBackRightDriveMotorId,
        TunerConstants.kBackRightSteerMotorId,
        TunerConstants.kBackRightEncoderOffset);


    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public class DriveSubsystem extends SubsystemBase {
        public DriveSubsystem() {
            AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

  }

public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

public void resetPose(Pose2d pose) {
    odometry.resetPosition( BlueAlliancePerspectiveRotation, m_modulePositions, pose);
}

// public ChassisSpeeds getRobotRelativeSpeeds() {
//     return odometry.getChassisSpeeds();
// }

// public void driveRobotRelative(ChassisSpeeds speeds) {
//     setControl(speeds);
// }

 public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    var swerveModuleStates = 
        TunerConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
  
    SwerveDriveKinematics.desaturateWheelSpeeds(
        (SwerveModuleState[]) swerveModuleStates, TunerConstants.kSpeedAt12VoltsMps);
  
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }
  
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return TunerConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }

}
    
    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }
    // public static double getKsimloopperiod() {
    //     return kSimLoopPeriod;
    // }
    // public Notifier getM_simNotifier() {
    //     return m_simNotifier;
    // }
    // public void setM_simNotifier(Notifier m_simNotifier) {
    //     this.m_simNotifier = m_simNotifier;
    // }
    // public double getM_lastSimTime() {
    //     return m_lastSimTime;
    // }
    // public void setM_lastSimTime(double m_lastSimTime) {
    //     this.m_lastSimTime = m_lastSimTime;
    // }
    // public SwerveDriveOdometry getOdometry() {
    //     return odometry;
    // }
    // public void setOdometry(SwerveDriveOdometry odometry) {
    //     this.odometry = odometry;
    // }
    // public Rotation2d getBlueAlliancePerspectiveRotation() {
    //     return BlueAlliancePerspectiveRotation;
    // }
    // public Rotation2d getRedAlliancePerspectiveRotation() {
    //     return RedAlliancePerspectiveRotation;
    // }
    // public boolean isHasAppliedOperatorPerspective() {
    //     return hasAppliedOperatorPerspective;
    // }
    // public void setHasAppliedOperatorPerspective(boolean hasAppliedOperatorPerspective) {
    //     this.hasAppliedOperatorPerspective = hasAppliedOperatorPerspective;
    // }
}
