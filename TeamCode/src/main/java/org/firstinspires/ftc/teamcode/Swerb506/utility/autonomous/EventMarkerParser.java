package org.firstinspires.ftc.teamcode.Swerb506.utility.autonomous;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Swerb506.opmode.autonomous.Events;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

public class EventMarkerParser {

    public List<EventMarker> parseEventMarkersFromPath(String fileName) {
        List<EventMarker> eventMarkers = new ArrayList<>();

        try {
            // Use AppUtil to get the file in the settings directory
            File file = AppUtil.getInstance().getSettingsFile(fileName + ".path");

            if (!file.exists()) {
                System.err.println("File not found: " + file.getAbsolutePath());
                return eventMarkers;
            }

            // Open the file
            try (InputStream inputStream = new FileInputStream(file);
                 BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream))) {

                StringBuilder fileContentBuilder = new StringBuilder();
                String line;
                while ((line = reader.readLine()) != null) {
                    fileContentBuilder.append(line);
                }

                String fileContent = fileContentBuilder.toString();
                JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

                JSONArray markersJson = (JSONArray) json.get("eventMarkers");
                for (Object markerJson : markersJson) {
                    EventMarker marker = fromJson((JSONObject) markerJson);
                    if (marker != null) {
                        eventMarkers.add(marker);
                    }
                }

            } catch (Exception e) {
                System.err.println("Error reading or parsing file: " + e.getMessage());
                e.printStackTrace();
            }

        } catch (Exception e) {
            System.err.println("Error accessing file: " + e.getMessage());
            e.printStackTrace();
        }

        return eventMarkers;
    }

    // Create an EventMarker object from JSON.
    public EventMarker fromJson(JSONObject json) {
        try {
            double relativePosition = ((Number) json.get("relativePosition")).doubleValue();
            String name = (String) json.get("name");

            // Assuming you have a way to map action names to Runnable instances.
            // Placeholder for action mapping logic:
            // Runnable action = eventRegistry.getEventActions().get(name);
            Runnable action = () -> {
                try {
                    Events.class.getMethod(name).invoke(null);
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }
            };
            return new EventMarker(relativePosition, action, name);
        } catch (Exception e) {
            System.err.println("Error parsing EventMarker from JSON: " + e.getMessage());
            e.printStackTrace();
            return null;
        }
    }
}