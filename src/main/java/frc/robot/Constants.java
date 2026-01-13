package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {

    public static final double kAutoAimDistance = 1; // meters
    public static final double kAutoAimDriveKP = 1;
    public static final double kAutoAimAngleKP = 0.01;
    
    public static final double ROBOT_MASS = Units.lbsToKilograms(120);
    public static final double MAX_SPEED = Units.feetToMeters(14.5);

    public static class ControllerConstants{
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        
        public static final double DEADBAND = 0.1;

    }

    public static class ShooterConstants{
        public static final int kLeaderMotorId = 15;
        //no slave, so thats it
    }

    public static class HoodConstants {
        public static final int kHoodMotorId = 19;
    }

}

