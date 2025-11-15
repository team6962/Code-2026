package com.team6962.lib.logging;

import java.util.Map;

import com.ctre.phoenix6.controls.ControlRequest;

import dev.doglog.DogLog;

public class LoggingUtil {
    public static void log(String path, ControlRequest controlRequest) {
        if (!path.endsWith("/")) {
            path += "/";
        }
        
        if (controlRequest == null) {
            return;
        }

        Map<String, String> controlInfo = controlRequest.getControlInfo();

        for (String field : controlRequest.getControlInfo().keySet()) {
            DogLog.log(path + field, controlInfo.get(field));
        }
    }
}
