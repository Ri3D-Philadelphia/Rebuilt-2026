package frc.robot.subsystems.fuelIntakeRoller;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class FuelIntakeRollerIOSpark implements FuelIntakeRollerIO {
    private SparkMax rollerMotor = new SparkMax(FuelIntakeRollerConstants.kRollerMotorId, MotorType.kBrushless);

    public FuelIntakeRollerIOSpark() {
        configureMotors();
    }
    
    /**
     * Configures the SparkMax.
     */
    private void configureMotors() {
        rollerMotor.configure( // configures single motor
            new SparkMaxConfig()
                .inverted(FuelIntakeRollerConstants.kInverted)
                .voltageCompensation(FuelIntakeRollerConstants.kNominalVoltage.in(Volts))
                .smartCurrentLimit(FuelIntakeRollerConstants.kCurrentLimit),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    /**
     * Set the SparkMax voltage.
     */
    public void setRollerVoltage(Voltage volts) {
        rollerMotor.setVoltage(volts);
    }

}
