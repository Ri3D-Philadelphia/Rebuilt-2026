package frc.robot.subsystems.drivetrain;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;

@Logged
public interface SwerveDriveIO {

    default void driveFieldCentric(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {}

    public Angle getFrontLeftAngle();
    
    public Angle getFrontRightAngle();
    
    public Angle getBackRightAngle();
    
    public Angle getBackLeftAngle();

}
