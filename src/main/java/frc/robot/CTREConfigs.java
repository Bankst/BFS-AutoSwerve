package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    private static final class Container {
        public static final CTREConfigs INSTANCE = new CTREConfigs();
    }

    public static CTREConfigs Get() {
        return Container.INSTANCE;
    }

    public final TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public final CANCoderConfiguration swerveCanCoderConfig = new CANCoderConfiguration();

    private CTREConfigs() {
        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
                Constants.SwerveK.kAngleEnableCurrentLimit,
                Constants.SwerveK.kAngleContinuousCurrentLimit,
                Constants.SwerveK.kAnglePeakCurrentLimit,
                Constants.SwerveK.kAnglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = Constants.SwerveK.kAngleKP;
        swerveAngleFXConfig.slot0.kI = Constants.SwerveK.kAngleKI;
        swerveAngleFXConfig.slot0.kD = Constants.SwerveK.kAngleKD;
        swerveAngleFXConfig.slot0.kF = Constants.SwerveK.kAngleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
                Constants.SwerveK.kDriveEnableCurrentLimit,
                Constants.SwerveK.kDriveContinuousCurrentLimit,
                Constants.SwerveK.kDrivePeakCurrentLimit,
                Constants.SwerveK.kDrivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.SwerveK.kDriveKP;
        swerveDriveFXConfig.slot0.kI = Constants.SwerveK.kDriveKI;
        swerveDriveFXConfig.slot0.kD = Constants.SwerveK.kDriveKD;
        swerveDriveFXConfig.slot0.kF = Constants.SwerveK.kDriveKF;
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.SwerveK.kOpenLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.SwerveK.kClosedLoopRamp;

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.SwerveK.kInvertCanCoder;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}