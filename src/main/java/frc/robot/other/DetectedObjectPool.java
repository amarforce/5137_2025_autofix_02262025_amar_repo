package frc.robot.other;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import edu.wpi.first.math.geometry.Pose3d;

public class DetectedObjectPool {
    private static final int POOL_SIZE = 50;
    private final Queue<DetectedObject> pool;
    
    public DetectedObjectPool() {
        pool = new ConcurrentLinkedQueue<>();
        // Pre-populate pool
        for (int i = 0; i < POOL_SIZE; i++) {
            pool.offer(new DetectedObject(new Pose3d(), 0, 0));
        }
    }
    
    public DetectedObject acquire(Pose3d pose, int classId, double detectionTime) {
        DetectedObject obj = pool.poll();
        if (obj == null) {
            obj = new DetectedObject(pose, classId, detectionTime);
        } else {
            obj.update(pose, classId, detectionTime);
        }
        return obj;
    }
    
    public void release(DetectedObject obj) {
        pool.offer(obj);
    }
}