package frc.robot.util;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Class to handle sending points through NetworkTables to be display on the
 * dashboard.
 * Use the static methods to register points. Rotation will currently not be
 * handled at all,
 * although in the future support for rotating points may be added.
 * Note that the display points currently are not implemented and are subject to
 * change.
 */
public class FieldPointDisplay implements Sendable {
    /// Class to store setter and getter methods.
    private static class PointData {
        public Supplier<Pose2d> get;
        public Consumer<Pose2d> set;

        public PointData(Supplier<Pose2d> getPose, Consumer<Pose2d> setPose) {
            get = getPose;
            set = setPose;
        }
    }

    static final ArrayList<String> setpointNames = new ArrayList<>();
    static final ArrayList<PointData> setpoints = new ArrayList<>();
    // Currently unimplemented. Would be for constant points that don't need a
    // setter and may change often.
    static ArrayList<Pose2d> displayPoints = new ArrayList<>();

    /**
     * Registers a setpoint to be displayed and edited. The getter will be called
     * to update
     * the point's positions and the setter will be called whenever the setpoint is
     * adjusted in
     * NetworkTables or on the dashboard.
     */
    public static void registerSetpoint(String name, Supplier<Pose2d> getter, Consumer<Pose2d> setter) {
        setpointNames.add(name);
        setpoints.add(new PointData(getter, setter));
    }

    /**
     * Remove a setpoint so it is no longer shown.
     */
    public static void removeSetpoint(String name) {
        int idx = setpointNames.indexOf(name);
        if (idx >= 0) {
            setpointNames.remove(idx);
            setpoints.remove(idx);
        }
    }

    /**
     * Clears the list of displayed constant points so none are shown.
     */
    public static void clearDisplayPointsList() {
        displayPoints.clear();
    }

    /**
     * Get the list of displayed constant points, to modify or remove any.
     */
    public static ArrayList<Pose2d> getDisplayPointsList() {
        return displayPoints;
    }

    /**
     * Set the list of displayed constant points.
     */
    public static void setDisplayPoints(ArrayList<Pose2d> points) {
        displayPoints = points;
    }

    private double[] getDisplayPointArray() {
        double[] arr = new double[displayPoints.size() * 2];
        for (int i = 0; i < displayPoints.size(); i++) {
            arr[i] = displayPoints.get(i).getX();
            arr[i + 1] = displayPoints.get(i).getY();
        }

        return arr;
    }

    private double[] getSetpointArray() {
        double[] arr = new double[setpoints.size() * 2];
        for (int i = 0; i < setpoints.size(); i++) {
            Pose2d p = setpoints.get(i).get.get();
            arr[i] = p.getX();
            arr[i + 1] = p.getY();
        }

        return arr;
    }

    private void unpackSetpointArray(double[] arr) {
        for (int i = 0; i < arr.length - 1; i += 2) {
            PointData data = setpoints.get(i / 2);
            Pose2d old = data.get.get();
            data.set.accept(new Pose2d(arr[i], arr[i + 1], old.getRotation()));
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleArrayProperty("setpoints", this::getSetpointArray, this::unpackSetpointArray);
        builder.addStringArrayProperty("setpointNames", () -> setpointNames.toArray(new String[0]), null);
        builder.addDoubleArrayProperty("displayPoints", this::getDisplayPointArray, null);
    }

}