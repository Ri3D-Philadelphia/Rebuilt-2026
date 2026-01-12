package frc.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

@Logged
public class ClimbInputs {
    /**
     * Current angle/position of climb motor.
     */
    public Angle climbAngle; 

    /**
     * Current (angular) velocity of climb motor.
     */
    public AngularVelocity climbVelocity;

    /**
     * Current of climb motor.
     */
    public Current climbCurrent;
}
