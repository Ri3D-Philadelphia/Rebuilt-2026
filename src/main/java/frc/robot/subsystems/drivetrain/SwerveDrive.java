package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class SwerveDrive extends SubsystemBase {

    private SwerveDriveIOSpark io;
    private Pigeon2Gyro gyro;
    
    public SwerveDrive() {
        this.io = new SwerveDriveIOSpark();
    }

    String testtt = "";
    double testt2 = 0;




    // TEST
    public Command GO() {
        return run(() -> io.powerFrontLeftDrive(Volts.of(2)));
    }
    
    public Command STOP() {
        return run(() -> io.powerFrontLeftDrive(Volts.of(0)));
    }

    /**
     * Set the motor speeds to match the given input velocities.
     */
    public Command driveFieldCentric(DoubleSupplier vxMetersPerSecond, DoubleSupplier vyMetersPerSecond, DoubleSupplier omegaRadiansPerSecond) {
        testtt = vxMetersPerSecond + "," + vyMetersPerSecond + "," + omegaRadiansPerSecond;
        testt2 = Math.random();
        return run(
            () -> io.driveFieldCentric(
                vxMetersPerSecond.getAsDouble(), 
                vyMetersPerSecond.getAsDouble(), omegaRadiansPerSecond.getAsDouble()
            )
        );
    }

    // ... existing code ...
    
    /**
     * Reset gyro to 0 degrees and update pose
     */
    public void resetGyro() {
        gyro.zeroHeading();
        
        System.out.println("Gyro reset to 0 degrees");
    }

    /**
     * Get gyro heading
     */
    public Rotation2d getHeading() {
        return gyro.getHeading();
    }
    
}
