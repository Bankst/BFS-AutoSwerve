package frc.lib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.lib.swerve.SwerveModuleMap.ModulePoseMap;
import frc.lib.swerve.SwerveModuleMap.ModulePosition;

public class SwerveDriveState {
    private Pose2d m_robotPose;
    private ModulePoseMap m_ModulePoseMap = new ModulePoseMap();

    private final Translation2d[] m_moduleTranslations;

    public SwerveDriveState(Translation2d[] moduleTranslations) {
        m_moduleTranslations = moduleTranslations;
    }

    public Pose2d getRobotPose() {
        return m_robotPose;
    }

    public SwerveModuleMap.ModulePoseMap getModulePoseMap() {
        return m_ModulePoseMap;
    }

    /**
     * 
     * @param robotPose Current robot pose
     * @param moduleStates All four module states, expected in FL, FR, RL, RR order
     */
    public void update(Pose2d robotPose, SwerveModuleState[] moduleStates) {
        m_robotPose = robotPose;

        for (ModulePosition p : ModulePosition.values()) {
            int i = p.idx;
            var modState = moduleStates[i];

            var modTranslation = m_moduleTranslations[i]
                .rotateBy(m_robotPose.getRotation())
                .plus(m_robotPose.getTranslation());
            var modPose = new Pose2d(modTranslation, modState.angle.plus(m_robotPose.getRotation()));

            m_ModulePoseMap.put(p, modPose);
        }
    }

    public void update(Pose2d robotPose, SwerveModuleState[] moduleStates, Field2d field) {
        update(robotPose, moduleStates);

        field.getObject("Robot").setPose(robotPose);
        field.getObject("Swerve Modules")
        .setPoses(SwerveModuleMap.orderedValues(m_ModulePoseMap, new Pose2d[0]));
    }
}