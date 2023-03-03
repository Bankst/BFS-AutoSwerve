package frc.robot;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final String canbus = "Canivore";

    public static final class SwerveK {
        public static final String DB_TAB_NAME = "SwerveSubsys";

        /**
         * Set to true to use external CANcoder for inital zero and switch to internal
         * falcon encoder for angle control.
         * Set to false to always use external CANcoder for angle control.
         * Recommended to set to false and always use CANCoder.
         */
        public static final boolean kUseInternalEncoder = false;

        public static final int kPigeonCANID = 1;
        public static final boolean kInvertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants kSwerveModule = COTSFalconSwerveConstants
                .SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants in meters */
        public static final double kTrackWidth = 0.622;
        public static final double kWheelBase = 0.5207;
        public static final double kWheelCircumference = kSwerveModule.wheelCircumference;

        /* Swerve Kinematics */
        public static final Translation2d[] kModuleTranslations = {
                new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)
        };

        public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(kModuleTranslations);

        /* Module Gear Ratios */
        public static final double kDriveGearRatio = kSwerveModule.driveGearRatio;
        public static final double kAngleGearRatio = kSwerveModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean kInvertAngleMotor = kSwerveModule.angleMotorInvert;
        public static final boolean kInvertDriveMotor = kSwerveModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean kInvertCanCoder = kSwerveModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int kAngleContinuousCurrentLimit = 25;
        public static final int kAnglePeakCurrentLimit = 40;
        public static final double kAnglePeakCurrentDuration = 0.1;
        public static final boolean kAngleEnableCurrentLimit = true;

        public static final int kDriveContinuousCurrentLimit = 35;
        public static final int kDrivePeakCurrentLimit = 60;
        public static final double kDrivePeakCurrentDuration = 0.1;
        public static final boolean kDriveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double kOpenLoopRamp = 0.25;
        public static final double kClosedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double kAngleKP = kSwerveModule.angleKP;
        public static final double kAngleKI = kSwerveModule.angleKI;
        public static final double kAngleKD = kSwerveModule.angleKD;
        public static final double kAngleKF = kSwerveModule.angleKF;

        /* Drive Motor PID Values */
        public static final double kDriveKP = 0.10;
        public static final double kDriveKI = 0.0;
        public static final double kDriveKD = 0.0;
        public static final double kDriveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double kDriveKS = 0.3 / 12; // TODO: This must be tuned to specific robot
        public static final double kDriveKV = 1.0 / 12;
        public static final double kDriveKA = 0.2 / 12;

        /* Feedforwards */
        public static final SimpleMotorFeedforward kDriveFF = new SimpleMotorFeedforward( // real
                kDriveKS, // Voltage to break static friction
                kDriveKV, // Volts per meter per second
                kDriveKA // Volts per meter per second squared
        );
        // Steer feed forward
        public static final SimpleMotorFeedforward kSteerFF = new SimpleMotorFeedforward( // real
                0.5, // Voltage to break static friction
                0.23, // Volts per radian per second
                0.0056 // Volts per radian per second squared
        );

        /* Swerve Profiling Values */
        public static final double kMaxVelocityMps = 8; // TODO: This must be tuned to specific robot //4.5
        /* Radians per Second */
        public static final double kMaxAngularVelocityRadps = 14; // TODO: This must be tuned to specific robot    //11.5

        /* Neutral Modes */
        public static final NeutralMode kAngleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode kDriveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(3.7);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(148.00);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 7;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(73.091);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back right Module - Module 3 */
        public static final class Mod3 {

            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 5;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(40.919);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 8;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        // weight for trusting vision over odometry (higher value = less trust)
        // currently unused
        public static final Matrix<N3, N1> kVisionStdDevs_DefaultTrust = VecBuilder.fill(0.9, 0.9, 0.9);
        public static final Matrix<N3, N1> kVisionStdDevs_NoTrust = VecBuilder.fill(100, 100, 100);

        public static double kPXController = 8;
        public static double kPYController = 8;
        public static double kPThetaController = 6.75;
        public static final double kDThetaController = 0.1;
        public static final double kFThetaController = 1;

        public static final double kOffBalanceAngleThresholdDegrees = Math.toRadians(10);
        public static final double kOnBalanceAngleThresholdDegrees = Math.toRadians(5);
        public static final double kMinimumBalanceDegrees = 2;

        public static final double kAlignAngleThresholdRadians = Math.toRadians(2.5);

        // Constraint for the motion profiled robt angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        // used for PPSwerve Auto Builder
        public static final PIDConstants kTranslationPID = new PIDConstants(kPXController, 0, 0); // x & y
        public static final PIDConstants kRotationPID = new PIDConstants(kPThetaController, 0, kDThetaController);
    }

  
}
