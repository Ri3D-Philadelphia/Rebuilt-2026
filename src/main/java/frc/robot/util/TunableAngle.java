/* (C) Robolancers 2025 */
package frc.robot.util;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

public class TunableAngle {

  private final Supplier<Angle> getter;

  public TunableAngle(String key, Angle defaultValue) {
    // if and only if no constant is there already, set it to the default value
    SmartDashboard.putNumber("/Tuning" + key, SmartDashboard.getNumber(key, defaultValue.in(Degrees)));

    this.getter = () -> Degrees.of(
      SmartDashboard.getNumber("/Tuning" + key, defaultValue.in(Degrees))
    );
  }

  public Angle get() {
    return this.getter.get();
  }
}