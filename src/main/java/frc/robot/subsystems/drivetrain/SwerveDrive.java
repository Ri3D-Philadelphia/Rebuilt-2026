package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.vision.AprilTagReceiver;

@Logged
public class SwerveDrive extends SubsystemBase {

    private double e_theta = 0;
    private double e_x = 0;
    private double e_y = 0;
    private double vxMetersPerSecond = 0;
    private double vyMetersPerSecond = 0;
    private double omegaRadiansPerSecond = 0;
        
    private SwerveDriveIOSpark io;
    
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
        return runOnce(
            () -> io.driveFieldCentric(
                vxMetersPerSecond.getAsDouble(), 
                vyMetersPerSecond.getAsDouble(), omegaRadiansPerSecond.getAsDouble()
            )
        );
    }

    /**
     * Auto-align the robot given april tag data
     * @param tagData
     * @return
     */
    public Command driveFieldCentric(Supplier<double[]> tagDataSupplier, Supplier<Rotation2d> robotYawSupplier) {
        // unpack data
        double[] tagData = tagDataSupplier.get();
        if (tagData.length < 5) {
            e_x = tagData.length;
            return driveFieldCentric(() -> 0, () -> 0, () -> 0);
        }

        double c_x = tagData[0];
        double c_y = tagData[1];
        double x_t = tagData[2];
        double y_t = tagData[3];
        double theta_t = tagData[4];
        double theta_h = robotYawSupplier.get().getRadians();
        
        if (c_x < 0 || c_y < 0) {
            e_y = -Math.random();
            return driveFieldCentric(() -> 0, () -> 0, () -> 0); // tag invalid
        } 

        /// get desired pose & errors
        // tag in robot POV
        double e_theta_prime = theta_t;
        double theta_proj = -(Math.PI - theta_t);
        double e_x_prime = x_t + Constants.kAutoAimDistance * Math.cos(theta_proj);
        double e_y_prime = y_t + Constants.kAutoAimDistance * Math.sin(theta_proj);
        // tag in field POV
        double cos = Math.cos(theta_h);
        double sin = Math.sin(theta_h);
        e_theta = e_theta_prime;
        e_x = e_x_prime*cos - e_y_prime*sin;
        e_y = e_x_prime*sin + e_y_prime*cos;

        SmartDashboard.putNumber("AutoAlign e_theta", e_theta);
        SmartDashboard.putNumber("AutoAlign e_x", e_x);
        SmartDashboard.putNumber("AutoAlign e_y", e_y);

        /// outputs
        // translation
        vxMetersPerSecond = Constants.kAutoAimDriveKP * e_x;
        vyMetersPerSecond = Constants.kAutoAimDriveKP * e_y;
        // rotation
        omegaRadiansPerSecond = Constants.kAutoAimAngleKP * -e_theta;
        
        SmartDashboard.putNumber("AutoAlign vxMetersPerSecond", vxMetersPerSecond);
        SmartDashboard.putNumber("AutoAlign vyMetersPerSecond", vyMetersPerSecond);
        SmartDashboard.putNumber("AutoAlign omegaRadiansPerSecond", omegaRadiansPerSecond);

        return driveFieldCentric(() -> vxMetersPerSecond, () -> vyMetersPerSecond, () -> omegaRadiansPerSecond);
    }

    @Override
    public void periodic() {
        driveFieldCentric(() -> AprilTagReceiver.readVisionData(), () -> io.getHeading());
    }
    
}
