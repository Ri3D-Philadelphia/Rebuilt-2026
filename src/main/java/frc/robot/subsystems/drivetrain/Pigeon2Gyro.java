package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import static edu.wpi.first.units.Units.Degrees;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
/**
 * Small Pigeon2 wrapper implementing a simple gyro interface.
 */
public class Pigeon2Gyro {
  

   private final Pigeon2 pigeon;

  public Pigeon2Gyro(int deviceId) {
    pigeon = new Pigeon2(deviceId);

    // apply default config (optional)
    Pigeon2Configuration cfg = new Pigeon2Configuration();
    pigeon.getConfigurator().apply(cfg);

    // optionally zero at construction
    pigeon.getConfigurator().setYaw(0.0);
  }  
  /** Return heading as a Rotation2d (degrees). */
  public Rotation2d getHeading() {
    // In Phoenix6 the yaw StatusSignal may return an Angle object; convert to degrees.
    var yawAngle = pigeon.getYaw().getValue(); // might be an Angle
    double yawDeg;

    // If getValue() returns an Angle from WPILib:
    try {
      yawDeg = yawAngle.in(Degrees);
    } catch (NoSuchMethodError | ClassCastException e) {
      // fallback: if getValue() already returns a primitive double
      // (adjust this if your CTRE API differs)
      yawDeg = ((Number) yawAngle).doubleValue();
    }

    // If you find sign is reversed when testing, invert here:
    return Rotation2d.fromDegrees(yawDeg);
  }

  /** Zero the pigeon yaw. */
  public void zeroHeading() {
    pigeon.getConfigurator().setYaw(0.0);
  }

}