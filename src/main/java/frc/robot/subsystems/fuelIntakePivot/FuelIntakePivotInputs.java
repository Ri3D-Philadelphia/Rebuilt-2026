package frc.robot.subsystems.fuelIntakePivot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

@Logged
public class FuelIntakePivotInputs {
    /**
     * Current angle of pivot.
     */
    public Angle pivotAngle; 

    /**
     * Current velocity of pivot.
     */
    public AngularVelocity pivotVelocity;

    /**
     * Current of pivot motor.
     */
    public Current pivotCurrent;
    
}
