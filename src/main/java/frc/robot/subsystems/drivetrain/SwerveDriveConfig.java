package frc.robot.subsystems.drivetrain;

import edu.wpi.first.epilogue.Logged;

@Logged
public record SwerveDriveConfig (
    double kP, 
    double kI, 
    double kD
) {}
