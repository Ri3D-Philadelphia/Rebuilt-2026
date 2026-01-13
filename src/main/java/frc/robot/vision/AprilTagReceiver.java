package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTagReceiver {

    public static double[] readVisionData() {
        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();

        // Get the "vision" table if it exists
        NetworkTable visionTable = ntInstance.getTable("vision");
        if (visionTable == null) {
            // System.out.println("No 'vision' table exists yet.");
            return new double[]{-1, -1};
        }

        // Check if "Tag Detected" exists and is true
        NetworkTableEntry tagDetectedEntry = visionTable.getEntry("Tag Detected");
        if (tagDetectedEntry == null) {
            // System.out.println("'Tag Detected' entry does not exist.");
            return new double[]{-1, -1};
        }

        boolean tagDetected = tagDetectedEntry.getBoolean(false); // default false if not yet set
        if (!tagDetected) {
            // System.out.println("No tag detected.");
            return new double[]{-1, -1};
        }

        // Check if "tag0" exists
        NetworkTableEntry tag0Entry = visionTable.getEntry("tag0");
        if (tag0Entry == null) {
            // System.out.println("'tag0' entry does not exist.");
            return new double[]{-1, -1};
        }

        double[] tag0Array = tag0Entry.getDoubleArray(new double[0]);
        if (tag0Array.length < 5) {
            // System.out.println("'tag0' array is missing or too short.");
            return new double[]{-1, -1};
        }

        // Extract x and y coordinates
        double cx = tag0Array[0];
        double cy = tag0Array[1];
        double xt = tag0Array[2];
        double yt = tag0Array[3];
        double yaw = tag0Array[4];

        return new double[]{cx, cy, xt, yt, yaw};
    }
    
}
