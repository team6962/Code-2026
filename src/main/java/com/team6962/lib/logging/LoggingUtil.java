package com.team6962.lib.logging;

import java.util.Map;

import com.ctre.phoenix6.controls.ControlRequest;

import dev.doglog.DogLog;

public class LoggingUtil {
    /**
     * Logs all fields from the given ControlRequest under the specified path.
     * 
     * @param path           The logging path under which to log the
     *                       ControlRequest fields.
     * @param controlRequest The ControlRequest whose fields are to be logged.
     */
    public static void log(String path, ControlRequest controlRequest) {
        if (!path.endsWith("/")) {
            path += "/";
        }
        
        if (controlRequest == null) {
            return;
        }

        Map<String, String> controlInfo = controlRequest.getControlInfo();

        for (String field : controlRequest.getControlInfo().keySet()) {
            String value = controlInfo.get(field);

            if (value.matches("-?\\d+(\\.\\d+)?")) {
                DogLog.log(path + field, Double.parseDouble(value));
            } else {
                DogLog.log(path + field, controlInfo.get(field));
            }
        }
    }

    /**
     * Returns the given logging path, ensuring that the returned path ends with
     * a slash.
     * 
     * @param path The logging path to ensure ends with a slash.
     * @return     The logging path, guaranteed to end with a slash.
     */
    public static String ensureEndsWithSlash(String path) {
        if (!path.endsWith("/")) {
            path += "/";
        }
        
        return path;
    }
}
