package frc.robot.subsystems.fuelIntakePivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class FuelIntakePivotIOSpark implements FuelIntakePivotIO {

    public static final FuelIntakePivotConfig config = new FuelIntakePivotConfig(
        0.00001, 
        0, 
        0, 
        0,
        Degrees.of(0), 
        Degrees.of(0)
    );

    private SparkMax pivotMotor = new SparkMax(
            FuelIntakePivotConstants.kPivotMotorId,
            MotorType.kBrushless
    );

    public FuelIntakePivotIOSpark() {
        configureMotors();
    }

    /**
     * Configures the SparkMax.
     */
    private void configureMotors() {
        pivotMotor.configure(
            new SparkMaxConfig()
                .inverted(FuelIntakePivotConstants.kInverted)
                .voltageCompensation(FuelIntakePivotConstants.kNominalVoltage.in(Volts))
                .smartCurrentLimit(FuelIntakePivotConstants.kCurrentLimit)
                .apply(
                    new EncoderConfig()
                        .velocityConversionFactor(
                            FuelIntakePivotConstants.kPivotVelocityConversionFactor)
                        .positionConversionFactor(
                            FuelIntakePivotConstants.kPivotPositionConversionFactor)),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    /**
     * Set motor voltage.
     */
    public void setPivotVoltage(Voltage volts) {
        pivotMotor.setVoltage(volts);
    }
    
    /**
     * Update the given inputs with the state of the pivot motor.
     */
    public void updateInputs(FuelIntakePivotInputs inputs) {
        inputs.pivotAngle = Degrees.of(pivotMotor.getEncoder().getPosition());
        inputs.pivotVelocity = DegreesPerSecond.of(pivotMotor.getEncoder().getVelocity());
        inputs.pivotCurrent = Amps.of(pivotMotor.getOutputCurrent());
    }
    
    /**
     * Reset the encoder to the given angle.
     */
    public void resetEncoder(Angle angle) {
        pivotMotor.getEncoder().setPosition(angle.in(Degrees));
    }
    
}
