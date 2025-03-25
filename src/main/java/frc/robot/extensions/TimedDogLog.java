package frc.robot.extensions;

import dev.doglog.DogLog;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

import java.security.InvalidParameterException;
import java.util.HashMap;
import java.util.Map;

public class TimedDogLog extends DogLog {

    private static final Map<String, Double> logTimeByKey = new HashMap<String, Double>();
    private static double defaultInterval = 0.500;

    private static boolean canLog(String key, double interval) {
        if (interval <= Constants.kRobotLoopTime) {
            throw new InvalidParameterException("The interval should be at least as long as the robot loop.");
        }
        double currentTime = Timer.getFPGATimestamp();
        double lastTime = logTimeByKey.getOrDefault(key, 0.0);
        double deltaTime = currentTime - lastTime;

        if (deltaTime >= interval) {
            logTimeByKey.put(key, currentTime);
            return true;
        }
        return false;
    }

    /* EXTENDED LOGGERS (WITH SPECIFIED INTERVALS) */
    //#region

    public static void log(String key, boolean value, double interval) {
        if (canLog(key, interval)) {
            DogLog.log(key, value);
        }
    }

    public static void log(String key, double[] value, double interval) {
        if (canLog(key, interval)) {
            DogLog.log(key, value);
        }
    }

    public static void log(String key, double value, double interval) {
        if (canLog(key, interval)) {
            DogLog.log(key, value);
        }
    }

    public static void log(String key, float[] value, double interval) {
        if (canLog(key, interval)) {
            DogLog.log(key, value);
        }
    }

    public static void log(String key, float value, double interval) {
        if (canLog(key, interval)) {
            DogLog.log(key, value);
        }
    }

    public static void log(String key, int[] value, double interval) {
        if (canLog(key, interval)) {
            DogLog.log(key, value);
        }
    }

    public static void log(String key, long[] value, double interval) {
        if (canLog(key, interval)) {
            DogLog.log(key, value);
        }
    }

    public static void log(String key, long value, double interval) {
        if (canLog(key, interval)) {
            DogLog.log(key, value);
        }
    }

    public static void log(String key, String[] value, double interval) {
        if (canLog(key, interval)) {
            DogLog.log(key, value);
        }
    }

    //#endregion


    /* DEFAULT INTERVAL LOGGERS */
    //#region
    public static void log(String key, Enum<?>[] value, double interval) {
        if (canLog(key, interval)) {
            DogLog.log(key, value);
        }
    }


    public static void log(String key, String value, double interval) {
        if (canLog(key, interval)) {
            DogLog.log(key, value);
        }
    }


    public static void log(String key, String value, String customTypeString, double interval) {
        if (canLog(key, interval)) {
            DogLog.log(key, value);
        }
    }


    public static void log(String key, Enum<?> value, double interval) {
        if (canLog(key, interval)) {
            DogLog.log(key, value);
        }
    }


    public static <T extends StructSerializable> void log(String key, T[] value, double interval) {
        if (canLog(key, interval)) {
            DogLog.log(key, value);
        }
    }


    public static <T extends StructSerializable> void log(String key, T value, double interval) {
        if (canLog(key, interval)) {
            DogLog.log(key, value);
        }
    }


    public static void log(String key, boolean value) {
        if (canLog(key, defaultInterval)) {
            DogLog.log(key, value);
        }
    }


    public static void log(String key, double[] value) {
        if (canLog(key, defaultInterval)) {
            DogLog.log(key, value);
        }
    }


    public static void log(String key, double value) {
        if (canLog(key, defaultInterval)) {
            DogLog.log(key, value);
        }
    }


    public static void log(String key, float[] value) {
        if (canLog(key, defaultInterval)) {
            DogLog.log(key, value);
        }
    }


    public static void log(String key, float value) {
        if (canLog(key, defaultInterval)) {
            DogLog.log(key, value);
        }
    }


    public static void log(String key, int[] value) {
        if (canLog(key, defaultInterval)) {
            DogLog.log(key, value);
        }
    }


    public static void log(String key, long[] value) {
        if (canLog(key, defaultInterval)) {
            DogLog.log(key, value);
        }
    }


    public static void log(String key, long value) {
        if (canLog(key, defaultInterval)) {
            DogLog.log(key, value);
        }
    }


    public static void log(String key, String[] value) {
        if (enabled) {
            var now = HALUtil.getFPGATime();
            logger.queueLog(now, key, value);
        }
    }

    public static void log(String key, Enum<?>[] value) {
        if (value == null) {
            return;
        }
        // Convert enum array to string array
        var stringArray = new String[value.length];

        for (int i = 0; i < value.length; i++) {
            stringArray[i] = value[i].name();
        }

        log(key, stringArray);
    }

    public static void log(String key, String value) {
        if (enabled) {
            var now = HALUtil.getFPGATime();
            logger.queueLog(now, key, value);
        }
    }

    public static void log(String key, String value, String customTypeString) {
        if (enabled) {
            var now = HALUtil.getFPGATime();
            logger.queueLog(now, key, value, customTypeString);
        }
    }

    /**
     * Log an enum. The enum will be converted to a string with {@link Enum#name()}.
     */
    public static void log(String key, Enum<?> value) {
        if (value == null) {
            return;
        }
        log(key, value.name());
    }

    /** Log a struct array. */
    public static <T extends StructSerializable> void log(String key, T[] value) {
        if (enabled) {
            var now = HALUtil.getFPGATime();
            logger.queueLog(now, key, value);
        }
    }

    /** Log a struct. */
    public static <T extends StructSerializable> void log(String key, T value) {
        if (enabled) {
            var now = HALUtil.getFPGATime();
            logger.queueLog(now, key, value);
        }
    }
    //#endregion

}
