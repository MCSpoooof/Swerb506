package org.firstinspires.ftc.teamcode.Swerb506.utility.autonomous;

import android.content.Context;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

public class EventMarkerParser {

    private Context context;

    public EventMarkerParser(Context context) {
        this.context = context;
    }

    public List<EventMarker> parseEventMarkersFromJson(String jsonPath) {
        List<EventMarker> markers = new ArrayList<>();

        try {
            JSONParser parser = new JSONParser();
            BufferedReader reader = new BufferedReader(new InputStreamReader(context.getAssets().open(jsonPath)));
            JSONObject jsonObject = (JSONObject) parser.parse(reader);
            JSONArray eventMarkers = (JSONArray) jsonObject.get("eventMarkers");

            for (Object markerObject : eventMarkers) {
                JSONObject markerJson = (JSONObject) markerObject;
                double position = ((Number) markerJson.get("waypointRelativePos")).doubleValue();
                String name = (String) markerJson.get("name");
                Runnable action = parseActionFromJson((JSONObject) markerJson.get("command"));
                markers.add(new EventMarker(position, action, name));
            }
        } catch (Exception e) {
            // Minimal logging for errors
            e.printStackTrace();
        }

        return markers;
    }

    private Runnable parseActionFromJson(JSONObject commandJson) {
        String type = (String) commandJson.get("type");
        return () -> {}; // Minimal action for efficiency
    }
}