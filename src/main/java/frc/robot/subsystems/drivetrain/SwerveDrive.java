package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class SwerveDrive extends SubsystemBase {

    private SwerveDriveIO io;
    
    public SwerveDrive() {
        this.io = new SwerveDriveIOSpark();
    }




    // TEST
    public Command GO(SparkMax mot) {
        return run(() -> mot.setVoltage(Volts.of(2)));
    }
    
    public Command STOP(SparkMax mot) {
        return run(() -> mot.setVoltage(Volts.of(0)));
    }

    /**
     * Set the motor speeds to match the given input velocities.
     */
    public Command driveFieldCentric(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        return new InstantCommand(
            () -> io.driveFieldCentric(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond), 
            this
        );
    }
    
}
