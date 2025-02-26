package frc.robot.other;

import edu.wpi.first.wpilibj.DataLogManager;

/**
 * MemoryManager class provides utilities for monitoring and managing memory usage.
 * It includes features for logging memory statistics, checking memory status,
 * and implementing log throttling to prevent excessive logging.
 */
public class MemoryManager {
    private static final Runtime runtime = Runtime.getRuntime();
    private static final long MB = 1024 * 1024;
    
    // Log throttling parameters
    private static final long DEFAULT_LOG_INTERVAL = 5000; // 5 seconds default interval
    private static long lastLogTime = 0;
    
    /**
     * Logs memory statistics if the minimum logging interval has elapsed.
     * This helps prevent excessive logging while still maintaining visibility
     * into memory usage patterns.
     */
    public static void logMemoryStats() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastLogTime < DEFAULT_LOG_INTERVAL) {
            return; // Skip logging if not enough time has passed
        }
        lastLogTime = currentTime;
        
        long total = runtime.totalMemory() / MB;
        long free = runtime.freeMemory() / MB;
        long max = runtime.maxMemory() / MB;
        long used = (runtime.totalMemory() - runtime.freeMemory()) / MB;
        
        // Only log if memory usage is above 75% or if it's a scheduled interval log
        if (used > (0.75 * max) || (currentTime / DEFAULT_LOG_INTERVAL) % 12 == 0) {
            DataLogManager.log(String.format(
                "Memory Stats - Used: %d MB, Free: %d MB, Total: %d MB, Max: %d MB",
                used, free, total, max
            ));
        }
    }
    
    /**
     * Gets the current used memory in bytes.
     * @return The amount of used memory in bytes
     */
    public static long getUsedMemory() {
        return runtime.totalMemory() - runtime.freeMemory();
    }
    
    /**
     * Gets the maximum available memory in bytes.
     * @return The maximum memory in bytes
     */
    public static long getMaxMemory() {
        return runtime.maxMemory();
    }
    
    /**
     * Checks if memory usage is above the specified threshold.
     * @param threshold The threshold as a decimal (e.g., 0.75 for 75%)
     * @return true if memory usage is above the threshold
     */
    public static boolean isMemoryLow(double threshold) {
        return getUsedMemory() > (threshold * getMaxMemory());
    }
    
    /**
     * Suggests garbage collection when memory usage is high.
     * Logs the event only if it hasn't been logged recently.
     */
    public static void suggestGC() {
        long currentTime = System.currentTimeMillis();
        System.gc();
        
        if (currentTime - lastLogTime >= DEFAULT_LOG_INTERVAL) {
            DataLogManager.log("Garbage collection suggested due to high memory usage");
            lastLogTime = currentTime;
        }
    }
}