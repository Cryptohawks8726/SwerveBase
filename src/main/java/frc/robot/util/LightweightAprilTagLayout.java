package frc.robot.util;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A lightweight alternative to the typical apriltag layout classes
 * <p>
 * Primarily useful when programming for unofficial games (i.e. Bunnybots by
 * 449)
 */
public class LightweightAprilTagLayout {
    // Internal dictionary for the tag layout
    private Map<Integer, Pose2d> apriltags;

    /**
     * Constructs a new layout with default map capacity & load factor.
     */
    LightweightAprilTagLayout() {
        apriltags = new HashMap<>(8, 1);
    }

    /**
     * Constructs a new layout
     * 
     * @param tagCount The amount of tags expected to be in the layout
     */
    LightweightAprilTagLayout(int tagCount) {
        apriltags = new HashMap<>(tagCount, 1);
    }

    /**
     * Adds an apriltag to the layout
     * 
     * @param id
     * @param tagPose Preferably blue origin
     * @return The new layout, for decorator purposes
     */
    public LightweightAprilTagLayout addApriltag(Integer id, Pose2d tagPose) {
        apriltags.put(id, tagPose);

        return this;
    }

    /**
     * Returns the pose of the tag with the given ID
     * 
     * @param id
     * @return Pose2d if the tag is in the layout, empty optional otherwise
     */
    public Optional<Pose2d> getTagPose(Integer id) {
        return Optional.ofNullable(apriltags.get(id));
    }

    /**
     * Gets the ID of every registered apriltag
     * 
     * @return An integer array of every registered apriltag
     */
    public int[] getIDs() {
        Set<Integer> objectArray = apriltags.keySet();
        int[] primitiveArray = new int[objectArray.size()];

        int i = 0;
        for (Integer val : objectArray) {
            primitiveArray[i] = val;
            i++;
        }

        return primitiveArray;
    }
}
