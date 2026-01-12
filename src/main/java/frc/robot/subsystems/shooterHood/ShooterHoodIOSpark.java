package frc.robot.subsystems.shooterHood;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;

@Logged
public class ShooterHoodIOSpark implements ShooterHoodIO {
    
    private final SparkMax hoodMotor;
    private final RelativeEncoder encoder;

    public ShooterHoodIOSpark() {
        hoodMotor = new SparkMax(ShooterHoodConstants.kHoodMotorId, MotorType.kBrushless);
        encoder = hoodMotor.getEncoder();
        configureMotors();
    }

    /**
     * Configures the SparkMax.
     */
    private void configureMotors() {
        hoodMotor.configure(
            new SparkMaxConfig()
                .inverted(ShooterHoodConstants.kInverted)
                .voltageCompensation(ShooterHoodConstants.kNominalVoltage.in(Volts))
                .smartCurrentLimit(ShooterHoodConstants.kCurrentLimit),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    /**
     * Update inputs with current motor state.
     */
    public void updateInputs(ShooterHoodInputs inputs) {
        double rotations = encoder.getPosition();
        inputs.angleDeg = rotations * ShooterHoodConstants.kDegreesPerRotation;
        inputs.appliedVolts = hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage();
        inputs.currentAmps = hoodMotor.getOutputCurrent();
    }

    /**
     * Set hood motor voltage.
     */
    public void setVoltage(double volts) {
        hoodMotor.setVoltage(volts);
    }

    /**
     * Stop hood motor.
     */
    public void stop() {
        hoodMotor.stopMotor();
    }
}