package org.firstinspires.ftc.teamcode.Swerb506.utility.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

public class EventMarkerParser {

    public List<EventMarker> parseEventMarkersFromPath(String fileName) {
        List<EventMarker> eventMarkers = new ArrayList<>();

        try {
            // Use AppUtil to get the file in the settings directory
            File file = AppUtil.getInstance().getSettingsFile(fileName + ".path");

            if (!file.exists()) {
                telemetry.addLine("File not found: " + file.getAbsolutePath());
                telemetry.update();
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
                telemetry.addLine("Error reading or parsing file: " + e.getMessage());
                telemetry.update();
                e.printStackTrace();
            }

        } catch (Exception e) {
            telemetry.addLine("Error accessing file: " + e.getMessage());
            telemetry.update();
            e.printStackTrace();
        }

        return eventMarkers;
    }

    // Create an EventMarker object from JSON.
    public EventMarker fromJson(JSONObject json) {
        try {
            double relativePosition = ((Number) json.get("relativePosition")).doubleValue();
            String name = (String) json.get("name");
            Runnable action = null;

            // Assuming you have a way to map action names to Runnable instances.
            // Placeholder for action mapping logic:
            // action = ActionRegistry.getAction(name);

            return new EventMarker(relativePosition, action, name);
        } catch (Exception e) {
            telemetry.addLine("Error parsing EventMarker from JSON: " + e.getMessage());
            telemetry.update();
            e.printStackTrace();
            return null;
        }
    }
}
