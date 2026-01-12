package frc.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;

@Logged
public record ClimbConfig(
    double kP, 
    double kI, 
    double kD, 
    double kG,
    Angle downAngle,
    Angle upAngle
) {}
