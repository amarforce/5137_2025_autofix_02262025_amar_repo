package frc.robot.other;

import edu.wpi.first.wpilibj.DataLogManager;

public class MemoryManager {
    private static final Runtime runtime = Runtime.getRuntime();
    private static final long MB = 1024 * 1024;
    
    public static void logMemoryStats() {
        long total = runtime.totalMemory() / MB;
        long free = runtime.freeMemory() / MB;
        long max = runtime.maxMemory() / MB;
        long used = (runtime.totalMemory() - runtime.freeMemory()) / MB;
        
        DataLogManager.log(String.format(
            "Memory Stats - Used: %d MB, Free: %d MB, Total: %d MB, Max: %d MB",
            used, free, total, max
        ));
    }
    
    public static long getUsedMemory() {
        return runtime.totalMemory() - runtime.freeMemory();
    }
    
    public static long getMaxMemory() {
        return runtime.maxMemory();
    }
    
    public static boolean isMemoryLow(double threshold) {
        return getUsedMemory() > (threshold * getMaxMemory());
    }
    
    public static void suggestGC() {
        System.gc();
        DataLogManager.log("Garbage collection suggested due to high memory usage");
    }
}