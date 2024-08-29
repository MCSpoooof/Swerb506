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
import java.util.ArrayList;
import java.util.List;

public class EventMarkerParser {

    public List<EventMarker> parseEventMarkersFromPath(String fileName) {
        List<EventMarker> eventMarkers = new ArrayList<>();

        try {
            // use AppUtil to get the file from the settings directory
            File file = AppUtil.getInstance().getSettingsFile(fileName + ".path");
            StringBuilder dataBuilder = new StringBuilder(); // this will build each line
            if (!file.exists()) { // kill the program if file is DNE
                System.err.println("File not found: " + file.getAbsolutePath());
                return eventMarkers;
            }

            // read through file and build
            try (InputStream inputStream = new FileInputStream(file);
                 BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream)))
            {
                String line;
                while ((line = reader.readLine()) != null) {
                    dataBuilder.append(line); // add each line to builder (more efficient concatenation)
                }
            } // parse data (as a String) into a JSONObject
            JSONObject fileContent = (JSONObject) new JSONParser().parse(dataBuilder.toString());

            // find the JSONArray of eventMarkers
            JSONArray markersJson = (JSONArray) fileContent.get("eventMarkers");
            for (Object markerJson : markersJson) { // loop through said array to find each marker
                EventMarker marker = createMarker((JSONObject) markerJson);
                if (marker != null) {
                    eventMarkers.add(marker);
                }
            }

        } catch (Exception e) {
            System.err.println("Error parsing file: " + e.getMessage());
            e.printStackTrace();        }

        return eventMarkers;
    }

    // create an EventMarker object from JSON.
    public EventMarker createMarker(JSONObject json) {
        try {
            // get the pose and name, cast them to Java types
            double relativePosition = (Double) json.get("relativePosition");
            String name = (String) json.get("name");

            // access the method (runnable) from Events based on "name"
            Runnable action = () -> {
                try {
                    Events.class.getMethod(name).invoke(null);
                } catch (Exception e) {
                    throw new RuntimeException("Failed to invoke" + name + "from Events: ", e);
                }
            };
            return new EventMarker(relativePosition, action, name);
        } catch (Exception e) {
            System.err.println("Error creating EventMarker from JSON: " + e.getMessage());
            e.printStackTrace();
            return null;
        }
    }
}