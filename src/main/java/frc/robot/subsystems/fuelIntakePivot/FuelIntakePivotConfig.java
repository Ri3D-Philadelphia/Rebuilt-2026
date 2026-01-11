package frc.robot.subsystems.fuelIntakePivot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;

@Logged
public record FuelIntakePivotConfig(
    double kP, 
    double kI, 
    double kD, 
    double kG,
    Angle downAngle,
    Angle upAngle
) {}
