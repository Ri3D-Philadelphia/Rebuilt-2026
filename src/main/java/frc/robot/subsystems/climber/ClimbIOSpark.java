package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Logged
public class ClimbIOSpark implements ClimbIO {

    public static final ClimbConfig config = new ClimbConfig(
        0.01,
        0, 
        0, 
        0,
        Degrees.of(0), 
        Degrees.of(90)
    );

    private SparkMax climbMotor = new SparkMax(
            ClimbConstants.kClimbMotorId,
            MotorType.kBrushless
    );

    public ClimbIOSpark() {
        configureMotors();
    }

    /**
     * Configures the SparkMax.
     */
    private void configureMotors() {
        climbMotor.configure(
            new SparkMaxConfig()
                .inverted(ClimbConstants.kInverted)
                .voltageCompensation(ClimbConstants.kNominalVoltage.in(Volts))
                .smartCurrentLimit(ClimbConstants.kCurrentLimit)
                .apply(
                    new EncoderConfig()
                        .velocityConversionFactor(
                            ClimbConstants.kClimbVelocityConversionFactor)
                        .positionConversionFactor(
                            ClimbConstants.kClimbPositionConversionFactor))
                .apply(
                    new SoftLimitConfig()
                        .forwardSoftLimit(ClimbConstants.kMaxRotations)
                        .forwardSoftLimitEnabled(true)
                        .reverseSoftLimit(ClimbConstants.kMinRotations)
                        .reverseSoftLimitEnabled(true)),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    /**
     * Set the voltage of the climb motor.
     */
    public void setClimbVoltage(Voltage volts) {
        climbMotor.setVoltage(volts);
    }
    
    /**
     * Update the given inputs with the state of the climb motor.
     */
    public void updateInputs(ClimbInputs inputs) {
        inputs.climbAngle = Degrees.of(climbMotor.getEncoder().getPosition());
        inputs.climbVelocity = DegreesPerSecond.of(climbMotor.getEncoder().getVelocity());
        inputs.climbCurrent = Amps.of(climbMotor.getOutputCurrent());
        SmartDashboard.putNumber("Climb Encoder",  (climbMotor.getEncoder().getPosition()));
    }

    /**
     * Reset the encoder to the given angle.
     */
    public void resetEncoder(Angle angle) {
        climbMotor.getEncoder().setPosition(angle.in(Degrees));
    }
    
}
