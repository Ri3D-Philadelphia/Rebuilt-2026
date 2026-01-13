/**
 * CODE STRUCTURE INSPIRED BY TEAM 321 REEFSCAPE
 */
package frc.robot.subsystems.fuelIntakePivot;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableAngle;
import frc.robot.util.TunableDouble;

@Logged
public class FuelIntakePivot extends SubsystemBase {

    public enum PivotState {
        EXTENDED,
        RETRACTED
    }
    
    private PivotState desiredState = PivotState.RETRACTED;

    private FuelIntakePivotIO io;
    private FuelIntakePivotInputs inputs;

    private PIDController fuelIntakeController; // pid 
    private ArmFeedforward feedForward; // ff

    private FuelIntakePivotConfig config;
  
    private boolean isHomed = false;

    private Debouncer homingDebouncer = new Debouncer(0.3, DebounceType.kBoth);

    public FuelIntakePivot() {
        this.io = new FuelIntakePivotIOSpark();
        this.config = FuelIntakePivotIOSpark.config;
        this.inputs = new FuelIntakePivotInputs();

        // pid & feedforward controller
        fuelIntakeController = new PIDController(config.kP(), config.kI(), config.kD());
        feedForward = new ArmFeedforward(0, config.kG(), 0); 

        fuelIntakeController.setTolerance(
            FuelIntakePivotConstants.kControllerTolerance.in(Degrees));

        io.resetEncoder(Degrees.of(0));
    }

    /**
     * Update this subsystem's pivot state to Extended.
     */
    public void setExtended() {
        desiredState = PivotState.EXTENDED;
    }

    /**
     * Update this subsystem's pivot state to Retracted.
     */
    public void setRetracted() {
        desiredState = PivotState.RETRACTED;
    }
    
    /**
     * Get whether the pivot is extended.
     */
    public boolean isExtended() {
        return desiredState == PivotState.EXTENDED;
    }

    /**
     * Set the pivot voltage based on state.
     */
    public Command holdPosition() {
        SmartDashboard.putString("Desired Pivot State", desiredState.toString());
        return run(() -> {
            if (desiredState == PivotState.EXTENDED) {
                runToAngle(config.downAngle());
            } else {
                runToAngle(config.upAngle());
            }
        });
    }

    /**
     * Tune the pivot motor using down angle as setpoint
     */
    public Command tune() {
        TunableDouble kP = new TunableDouble("/FuelIntakePivot/kP", config.kP());
        TunableDouble kI = new TunableDouble("/FuelIntakePivot/kI", config.kI());
        TunableDouble kD = new TunableDouble("/FuelIntakePivot/kD", config.kD());
        TunableDouble kG = new TunableDouble("/FuelIntakePivot/kG", config.kG());
        TunableAngle downAngle = new TunableAngle("/FuelIntakePivot/downAngle", config.downAngle());

        return run(() -> {
            this.fuelIntakeController.setPID(kP.get(), kI.get(), kD.get());
            this.feedForward = new ArmFeedforward(0, kG.get(), 0);
            runToAngle(downAngle.get());
        });
    }

    /**
     * Set motor control output using given angle.
     * @param desiredAngle
     */
    public void runToAngle(Angle desiredAngle) {
        Voltage desiredVoltage = Volts.of(
            feedForward.calculate(desiredAngle.in(Degrees), 0) + 
            fuelIntakeController.calculate(inputs.pivotAngle.in(Degrees), desiredAngle.in(Degrees))
        );

        SmartDashboard.putNumber("runToAngle target ang (deg)", desiredAngle.in(Degrees));
        SmartDashboard.putNumber("runToAngle current ang (deg)", inputs.pivotAngle.in(Degrees));
        SmartDashboard.putNumber("runToAngle output voltage", desiredVoltage.in(Volts));

        io.setPivotVoltage(desiredVoltage);
    }

    /**
     * Control motor to given angle.
     * @param desiredAngle
     */
    public Command runToAngle(Supplier<Angle> desiredAngle) {
        return run(() -> { runToAngle(desiredAngle.get()); });
    }
    
    /**
     * Return a command to move the intake down.
     */
    public Command moveDown() {
        return runToAngle(() -> config.downAngle());
    }
    
    /**
     * Return a command to move the intake up.
     */
    public Command moveUp() {
        return runToAngle(() -> config.upAngle());
    }

    /**
     * Return a command to home the intake's angle to 0.
     */
    public Command home() {
        return run(() -> { io.setPivotVoltage(FuelIntakePivotConstants.kHomeVoltage); })
            .until(
                () -> homingDebouncer.calculate(inputs.pivotCurrent.in(Amp) > FuelIntakePivotConstants.kHomeCurrentThresh.in(Amp))
            )
            .andThen(runOnce(
                () -> {
                    io.resetEncoder(Degrees.of(0));
                    isHomed = true;
                }
            )
        );
    }

    /**
     * Get the pivot intake angle.
     */
    public Angle getAngle() {
        return inputs.pivotAngle;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    /**
     * Return whether the pivot is at its setpoint.
     */
    public boolean atSetpoint() {
        return fuelIntakeController.atSetpoint();
    }

    /**
     * Get whether the pivot is homed.
     */
    public boolean isHomed() {
        return isHomed;
    }

    
}
