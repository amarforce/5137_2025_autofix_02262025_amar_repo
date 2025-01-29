package frc.robot.other;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.VisionConstants;

/**
 * Represents a detected object in 3D space, including its position and classification.
 */
public class DetectedObject {
    // The 3D position of the detected object.
    private Translation3d pos;

    // The class ID of the detected object, which corresponds to a specific type or category.
    private int classId;

    /**
     * Constructs a new DetectedObject with the specified position and class ID.
     *
     * @param pos The 3D position of the detected object.
     * @param classId The class ID of the detected object.
     */
    public DetectedObject(Translation3d pos, int classId) {
        this.pos = pos;
        this.classId = classId;
    }

    /**
     * Returns the 3D position of the detected object.
     *
     * @return The 3D position of the detected object.
     */
    public Translation3d getPos() {
        return pos;
    }

    /**
     * Returns the class ID of the detected object.
     *
     * @return The class ID of the detected object.
     */
    public int getClassId() {
        return classId;
    }

    /**
     * Returns the class name of the detected object based on its class ID.
     * The class name is retrieved from the VisionConstants.classNames array.
     *
     * @return The class name of the detected object.
     */
    public String getClassName() {
        return VisionConstants.classNames[classId];
    }
}