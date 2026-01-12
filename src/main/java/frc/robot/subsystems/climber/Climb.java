package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableAngle;
import frc.robot.util.TunableDouble;

@Logged
public class Climb extends SubsystemBase {

    private ClimbIO io;
    private ClimbInputs inputs;

    private PIDController climbController; // pid
    private ArmFeedforward feedForward; // ff

    private ClimbConfig config;

    private boolean isHomed = false;
    
    private Debouncer homingDebouncer = new Debouncer(0.3, DebounceType.kBoth);

    public Climb() {
        this.io = new ClimbIOSpark();
        this.config = ClimbIOSpark.config;
        this.inputs = new ClimbInputs();

        // pid & ff
        climbController = new PIDController(config.kP(), config.kI(), config.kD());
        feedForward = new ArmFeedforward(0, config.kG(), 0);

        climbController.setTolerance(
            ClimbConstants.kControllerTolerance.in(Degrees)
        );

        io.resetEncoder(Degrees.of(0));
    }

    public Command resetEncoder() {
        return runOnce(() -> io.resetEncoder(Degrees.of(0)));
    }

    /**
     * Tune the climb motor using down angle as setpoint.
     */
    public Command tune() {
        TunableDouble kP = new TunableDouble("/Climb/kP", config.kP());
        TunableDouble kI = new TunableDouble("/Climb/kI", config.kI());
        TunableDouble kD = new TunableDouble("/Climb/kD", config.kD());
        TunableDouble kG = new TunableDouble("/Climb/kG", config.kG());
        TunableAngle downAngle = new TunableAngle("/Climb/downAngle", config.downAngle());

        return run(() -> {
            this.climbController.setPID(kP.get(), kI.get(), kD.get());
            this.feedForward = new ArmFeedforward(0, kG.get(), 0);
            runToAngle(downAngle.get());
        });
    }

    /**
     * Set motor control output using given angle.
     */
    public void runToAngle(Angle desiredAngle) {
        Voltage desiredVoltage = Volts.of(
            feedForward.calculate(desiredAngle.in(Degrees), 0) + 
            climbController.calculate(inputs.climbAngle.in(Degrees), desiredAngle.in(Degrees))
        );

        SmartDashboard.putNumber("Target Climb Angle", desiredAngle.in(Degrees));
        SmartDashboard.putNumber("Curr Climb Angle", inputs.climbAngle.in(Degrees));

        io.setClimbVoltage(desiredVoltage);
    }

    /**
     * Control motor to given angle.
     * @param desiredAngle
     */
    public Command runToAngle(Supplier<Angle> desiredAngle) {
        return run(() -> { runToAngle(desiredAngle.get()); });
    }
    
    /**
     * Return a command to move the climb down.
     */
    public Command climbDown() {

        SmartDashboard.putBoolean("DownCalled", true);

        return runToAngle(() -> config.downAngle());
    }
    
    /**
     * Return a command to move the climb up.
     */
    public Command climbUp() {
        SmartDashboard.putBoolean("UpCalled", true);

        return Commands.run(() -> runToAngle(config.upAngle()), this);

    }

    /**
     * Return a command to home the climb motor to 0.
     */
    public Command home() {
        return run(() -> { io.setClimbVoltage(ClimbConstants.kHomeVoltage); })
            .until(
                () -> homingDebouncer.calculate(inputs.climbCurrent.in(Amp) > ClimbConstants.kHomeCurrentThresh.in(Amp))
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
     * Get the climb motor angle/position.
     */
    public Angle getAngle() {
        return inputs.climbAngle;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    /**
     * Return whether the climb is at its setpoint.
     */
    public boolean atSetpoint() {
        return climbController.atSetpoint();
    }

    /**
     * Get whether the climb is homed.
     */
    public boolean isHomed() {
        return isHomed;
    }

    /**
     *set voltage to 1
     */
    public Command setVoltage1() {
        return run(() -> io.setClimbVoltage(Volts.of(1)));
    }

    /**
     *set voltage to -1
     */
    public Command setVoltageminus1() {
        return run(() -> io.setClimbVoltage(Volts.of(-1)));
    }

    /**
     * Get whether the climb is homed.
     */
    public Command stop() {
        return run(() -> io.setClimbVoltage(Volts.of(0)));
    }



    
}
