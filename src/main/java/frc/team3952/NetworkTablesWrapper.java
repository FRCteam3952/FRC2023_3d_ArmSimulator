package frc.team3952;

import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.CombinedRuntimeLoader;
import edu.wpi.first.util.WPIUtilJNI;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class NetworkTablesWrapper {
    private static final NetworkTableInstance NT_INST = NetworkTableInstance.getDefault();
    private static final Map<String, GenericPublisher> publishers = new HashMap<>();
    private static final Map<String, GenericSubscriber> subscribers = new HashMap<>();

    /**
     * Gets the NetworkTables Instance being used by the program
     *
     * @return {@link NetworkTableInstance} used
     */
    public static NetworkTableInstance getNTInstance() {
        return NT_INST;
    }

    /**
     * Returns the table reference from NetworkTables
     *
     * @param tableName The name of the table
     * @return {@link NetworkTable} corresponding
     */
    public static NetworkTable getTable(String tableName) {
        return NT_INST.getTable(tableName);
    }

    /**
     * Returns the entry reference from NetworkTables
     *
     * @param tableName Name of the table
     * @param entryName Name of the entry
     * @return {@link NetworkTableEntry} corresponding
     */
    public static NetworkTableEntry getEntry(String tableName, String entryName) {
        return getTable(tableName).getEntry(entryName);
    }

    public static GenericPublisher getPublisher(String tableName, String entryName) {
        String path = "/" + tableName + "/" + entryName;
        var temp = publishers.get(path);
        if (temp != null) {
            return temp;
        }
        var entry = getEntry(tableName, entryName);
        var newPublisher = entry.getTopic().genericPublish(entry.getType().getValueStr(), PubSubOption.keepDuplicates(true));
        publishers.put(path, newPublisher);
        return newPublisher;
    }

    public static GenericSubscriber getSubscriber(String tableName, String entryName) {
        String path = "/" + tableName + "/" + entryName;
        var temp = subscribers.get(path);
        if (temp != null) {
            return temp;
        }
        var entry = getEntry(tableName, entryName);
        var newSubscriber = entry.getTopic().genericSubscribe(entry.getType().getValueStr(), PubSubOption.keepDuplicates(true), PubSubOption.pollStorage(10));
        subscribers.put(path, newSubscriber);
        return newSubscriber;
    }

    public static void init() throws IOException {
        NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
        WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
        WPIMathJNI.Helper.setExtractOnStaticLoad(false);

        CombinedRuntimeLoader.loadLibraries(Armvisualiser.class, "wpiutiljni", "wpimathjni", "ntcorejni");

        NT_INST.startClient4("Robot 3D GUI");
        NT_INST.setServer("localhost");
        NT_INST.startDSClient();
        System.out.println(NT_INST.isConnected());
    }

    public static Pose2d getJetsonPose() {
        double[] pose = getEntry("jetson", "apriltags_pose").getDoubleArray(new double[] {0, 0, 0});
        return new Pose2d(pose[0], pose[2], new Rotation2d());
    }

    public static Pose2d getRobotPose() {
        double[] pose = getEntry("robot", "drive_odometry").getDoubleArray(new double[] {0, 0, 0});
        return new Pose2d(pose[0], pose[1], new Rotation2d(pose[2]));
    }

    public static double[] getArmAngles() {
        return getEntry("robot", "arm_angles").getDoubleArray(new double[] {0, 0, 0});
    }

    public static double[] getArmTargetCoordinates() {
        return getEntry("robot", "arm_target_coords").getDoubleArray(new double[] {0, 0, 0});
    }
}
