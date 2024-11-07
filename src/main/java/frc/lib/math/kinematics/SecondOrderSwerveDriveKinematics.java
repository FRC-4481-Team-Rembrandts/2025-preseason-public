package frc.lib.math.kinematics;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SecondOrderSwerveDriveKinematics extends SwerveDriveKinematics {
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters, double Acceleration) {
        return super.toSwerveModuleStates(chassisSpeeds);
    }

    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds, double Acceleration){
        return super.toSwerveModuleStates(chassisSpeeds);
    }
}

