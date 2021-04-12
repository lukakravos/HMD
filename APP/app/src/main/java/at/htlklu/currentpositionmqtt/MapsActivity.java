package at.htlklu.currentpositionmqtt;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.fragment.app.FragmentActivity;

import android.Manifest;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.Dialog;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.graphics.drawable.ColorDrawable;
import android.location.Address;
import android.location.Geocoder;
import android.location.LocationManager;
import android.media.Image;
import android.os.Bundle;
import android.view.View;
import android.widget.ImageButton;
import android.widget.TextView;
import android.widget.Toast;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.LatLngBounds;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.Polygon;
import com.google.android.gms.maps.model.PolygonOptions;

import org.eclipse.paho.android.service.MqttAndroidClient;
import org.eclipse.paho.client.mqttv3.IMqttActionListener;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.IMqttToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;

import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.List;

public class MapsActivity extends FragmentActivity implements OnMapReadyCallback {

    private GoogleMap mMap;
    private Boolean startedMap = false, droneStarted = false, testI = true;
    private LocationManager locationManager;
    private MqttAndroidClient client;

    private String latLngDroneTopic = "htl/diplomarbeit/wildschutzdrohne/drone/position/latLng";
    private String latLngsFieldTopic = "htl/diplomarbeit/wildschutzdrohne/field/latLngs";
    private String altitudeDroneTopic = "htl/diplomarbeit/wildschutzdrohne/drone/altitude";
    private String headingDroneTopic = "htl/diplomarbeit/wildschutzdrohne/drone/heading";
    private String firmVerDroneTopic = "htl/diplomarbeit/wildschutzdrohne/drone/version";
    private String gpsDroneTopic = "htl/diplomarbeit/wildschutzdrohne/drone/gps_state";
    private String speedDroneTopic = "htl/diplomarbeit/wildschutzdrohne/drone/speed";
    private String batDroneTopic = "htl/diplomarbeit/wildschutzdrohne/drone/battery";
    private String modeDroneTopic = "htl/diplomarbeit/wildschutzdrohne/drone/mode";
    private String armedDroneTopic = "htl/diplomarbeit/wildschutzdrohne/drone/isArmed";

    private Boolean connected = false;
    private LatLng dronePos;
    private Marker markerDrone;
    private LatLngBounds.Builder builder;
    private LatLngBounds bounds;
    private Polygon fieldPoly;
    private ImageButton drZoom, shwAtt;

    private Dialog myDialog;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_maps);

        myDialog = new Dialog(MapsActivity.this);

        drZoom = (ImageButton) findViewById(R.id.droneZoom);
        shwAtt = (ImageButton) findViewById(R.id.showAttributes);

        connectToMqtt();

        initMap();
    }

    public void initMap() {
        // Obtain the SupportMapFragment and get notified when the map is ready to be used.
        SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager()
                .findFragmentById(R.id.map);
        mapFragment.getMapAsync(this);
    }

    private void connectToMqtt() {
        String clientId = MqttClient.generateClientId();

        MqttConnectOptions options = new MqttConnectOptions();
        options.setMqttVersion(MqttConnectOptions.MQTT_VERSION_3_1);
        options.setUserName("htl-IoT");
        options.setPassword("iot..2015".toCharArray());

        client =
                new MqttAndroidClient(MapsActivity.this, "tcp://iotmqtt.htl-klu.at:1883",
                        clientId);

        try {
            IMqttToken token = client.connect(options);
            token.setActionCallback(new IMqttActionListener() {
                @Override
                public void onSuccess(IMqttToken asyncActionToken) {
                    // We are connected
                    //Toast.makeText(MapsActivity.this, "connected", Toast.LENGTH_SHORT).show();
                    connected = true;
                    //subscribeToMqtt(latLngMeTopic);
                    subscribeToMqtt(latLngsFieldTopic);
                    subscribeToMqtt(latLngDroneTopic);
                    subscribeToMqtt(altitudeDroneTopic);
                    subscribeToMqtt(headingDroneTopic);
                    subscribeToMqtt(armedDroneTopic);
                    subscribeToMqtt(batDroneTopic);
                    subscribeToMqtt(gpsDroneTopic);
                    subscribeToMqtt(modeDroneTopic);
                    subscribeToMqtt(speedDroneTopic);
                    subscribeToMqtt(firmVerDroneTopic);
                    //Toast.makeText(MapsActivity.this, "subscribed", Toast.LENGTH_SHORT).show();

                    getMqttMessage();
                }

                @Override
                public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                    // Something went wrong e.g. connection timeout or firewall problems
                    Toast.makeText(MapsActivity.this, "Connection failed", Toast.LENGTH_SHORT).show();
                    connected = false;
                }
            });
        } catch (MqttException e) {
            e.printStackTrace();
        }
    }

    public void getMqttMessage() {
        client.setCallback(new MqttCallback() {
            @Override
            public void connectionLost(Throwable cause) {
                Toast.makeText(MapsActivity.this, "Mqtt connection lost", Toast.LENGTH_SHORT).show();
            }

            @Override
            public void messageArrived(String topic, MqttMessage message) throws Exception {
                String mess = new String(message.getPayload());
                //Toast.makeText(MapsActivity.this, mess, Toast.LENGTH_SHORT).show();
                if (topic.equalsIgnoreCase(latLngDroneTopic)) {
                    //Toast.makeText(MapsActivity.this, mess, Toast.LENGTH_SHORT).show();
                    setDronePosition(mess);
                } else if (topic.equalsIgnoreCase(latLngsFieldTopic)) {
                    //Toast.makeText(MapsActivity.this, mess, Toast.LENGTH_SHORT).show();
                    drawField(mess);
                } else if (topic.equalsIgnoreCase(altitudeDroneTopic)) {
                    Toast.makeText(MapsActivity.this, mess, Toast.LENGTH_SHORT).show();

                } else if (topic.equalsIgnoreCase(headingDroneTopic)) {
                    Toast.makeText(MapsActivity.this, mess, Toast.LENGTH_SHORT).show();

                } else if (topic.equalsIgnoreCase(armedDroneTopic)) {
                    Toast.makeText(MapsActivity.this, mess, Toast.LENGTH_SHORT).show();

                } else if (topic.equalsIgnoreCase(batDroneTopic)) {
                    Toast.makeText(MapsActivity.this, mess, Toast.LENGTH_SHORT).show();

                } else if (topic.equalsIgnoreCase(firmVerDroneTopic)) {
                    Toast.makeText(MapsActivity.this, mess, Toast.LENGTH_SHORT).show();

                } else if (topic.equalsIgnoreCase(gpsDroneTopic)) {
                    Toast.makeText(MapsActivity.this, mess, Toast.LENGTH_SHORT).show();

                } else if (topic.equalsIgnoreCase(modeDroneTopic)) {
                    Toast.makeText(MapsActivity.this, mess, Toast.LENGTH_SHORT).show();

                } else if (topic.equalsIgnoreCase(speedDroneTopic)) {
                    Toast.makeText(MapsActivity.this, mess, Toast.LENGTH_SHORT).show();

                } else {
                    Toast.makeText(MapsActivity.this, "topic wrong", Toast.LENGTH_SHORT).show();
                }
            }

            @Override
            public void deliveryComplete(IMqttDeliveryToken token) {
            }
        });
    }

    public void drawField(String message) {
        if (!message.contains(";") || !message.contains(",")) {
            Toast.makeText(MapsActivity.this, "No comma and/or semi", Toast.LENGTH_SHORT).show();
            return;
        }
        String[] latLngsStr = message.split(";");
        if (latLngsStr.length <= 2) {
            Toast.makeText(MapsActivity.this, "Too less points for field", Toast.LENGTH_SHORT).show();
            return;
        }
        try {
            LatLng[] latLngs = new LatLng[latLngsStr.length + 1];
            ArrayList<LatLng> lst = new ArrayList<LatLng>();
            for (int i = 0; i < latLngs.length - 1; i++) {
                String[] latLngStr = latLngsStr[i].split(",");
                if (latLngStr.length != 2) {
                    Toast.makeText(MapsActivity.this, "No lat or long", Toast.LENGTH_SHORT).show();
                    return;
                }
                latLngs[i] = new LatLng(Double.parseDouble(latLngStr[0]), Double.parseDouble(latLngStr[1]));
                //lst.add(latLngs[i]);
            }
            //latLngs[latLngs.length - 1] = latLngs[0];

            sort(latLngs, 0, latLngs.length - 2);

            if (latLngs[0].longitude < latLngs[1].longitude) {
                LatLng tmp = latLngs[0];
                latLngs[0] = latLngs[1];
                latLngs[1] = tmp;
            }

            if (latLngs[2].longitude > latLngs[3].longitude) {
                LatLng tmp = latLngs[2];
                latLngs[2] = latLngs[3];
                latLngs[3] = tmp;
            }

            latLngs[latLngs.length - 1] = latLngs[0];

            for (int i = 0; i < latLngs.length - 1; i++) {
                lst.add(latLngs[i]);
            }

            if (testI) {
                testI = false;
                PolygonOptions fieldPolyOptions = new PolygonOptions();
                fieldPolyOptions.fillColor(Color.argb(75, 255, 255, 0));
                fieldPolyOptions.strokeWidth(10f);
                fieldPolyOptions.strokeColor(Color.YELLOW);
                fieldPolyOptions.add(latLngs);
                fieldPoly = mMap.addPolygon(fieldPolyOptions);
                fieldPoly.setTag("Field");
                return;
            }

            fieldPoly.setPoints(lst);
        } catch (NumberFormatException e) {
            Toast.makeText(MapsActivity.this, "Error on Coords", Toast.LENGTH_SHORT).show();
            e.printStackTrace();
        }
    }

    public void sort(LatLng[] lngs, int low, int high) {
        if (low < high) {
            /* pi is partitioning index, arr[pi] is
              now at right place */
            int pi = partition(lngs, low, high);

            // Recursively sort elements before
            // partition and after partition
            sort(lngs, low, pi - 1);
            sort(lngs, pi + 1, high);
        }
    }

    public int partition(LatLng[] lngs, int low, int high) {
        double pivot = lngs[high].latitude;
        int i = (low - 1); // index of smaller element
        for (int j = low; j < high; j++) {
            // If current element is smaller than the pivot
            if (lngs[j].latitude < pivot) {
                i++;

                // swap arr[i] and arr[j]
                double temp = lngs[i].latitude;
                double tmp2 = lngs[i].longitude;
                lngs[i] = lngs[j];
                lngs[j] = new LatLng(temp, tmp2);
            }
        }

        // swap arr[i+1] and arr[high] (or pivot)
        double temp = lngs[i + 1].latitude;
        double tmp2 = lngs[i + 1].longitude;
        lngs[i + 1] = lngs[high];
        lngs[high] = new LatLng(temp, tmp2);

        return i + 1;
    }

    public void setDronePosition(String message) {
        //Toast.makeText(MapsActivity.this, "into topic", Toast.LENGTH_SHORT).show();
        if (message.contains(",")) { //46.451241234, 12.323265
            //Toast.makeText(MapsActivity.this, "into comma", Toast.LENGTH_SHORT).show();
            message.trim();
            String[] latLngString = message.split(",");
            //Toast.makeText(MapsActivity.this, "into split", Toast.LENGTH_SHORT).show();
            Geocoder geocoder = new Geocoder(getApplicationContext());
            //Toast.makeText(MapsActivity.this, "into geoce", Toast.LENGTH_SHORT).show();
            try {
                Double lat = Double.parseDouble(latLngString[0].trim());
                Double lng = Double.parseDouble(latLngString[1].trim());
                dronePos = new LatLng(lat, lng);

                //Toast.makeText(MapsActivity.this, "dronePos", Toast.LENGTH_SHORT).show();

                String str = "Unknown Location";

                List<Address> addressList = geocoder.getFromLocation(lat, lng, 1);
                if (!addressList.isEmpty()) {
                    str = addressList.get(0).getLocality() + ", ";
                    str += addressList.get(0).getCountryName();
                }

                if (!droneStarted) {
                    markerDrone = mMap.addMarker(new MarkerOptions().position(dronePos).title("Drone").snippet(str));
                    droneStarted = true;
                }

                markerDrone.setPosition(dronePos);
                markerDrone.setSnippet(str);
                //Toast.makeText(MapsActivity.this, "into snipp", Toast.LENGTH_SHORT).show();
                builder.include(markerDrone.getPosition());
                //Toast.makeText(MapsActivity.this, "into builder", Toast.LENGTH_SHORT).show();
                bounds = builder.build();
                //Toast.makeText(MapsActivity.this, "into bounds", Toast.LENGTH_SHORT).show();

                drZoom.setVisibility(View.VISIBLE);
                shwAtt.setVisibility(View.VISIBLE);

                return;
                //Toast.makeText(MapsActivity.this, "into try", Toast.LENGTH_SHORT).show();
            } catch (NumberFormatException | IOException e) {
                e.printStackTrace();
                Toast.makeText(MapsActivity.this, "Number wrong", Toast.LENGTH_SHORT).show();
            }
        } else {
            Toast.makeText(MapsActivity.this, "No komma", Toast.LENGTH_SHORT).show();
        }
        if (markerDrone.isVisible()) {
            markerDrone.setVisible(false);
            drZoom.setVisibility(View.INVISIBLE);
            shwAtt.setVisibility(View.INVISIBLE);
        }
    }

    public void centerOnPosition(LatLng latLng) {
        mMap.animateCamera(CameraUpdateFactory.newLatLngZoom(latLng, 15.2f));
    }

    public void publishToMqtt(String topic, String payload) {
        try {
            byte[] encodedPayload = payload.getBytes("UTF-8");
            client.publish(topic, encodedPayload, 0, false);
        } catch (UnsupportedEncodingException | MqttException e) {
            e.printStackTrace();
        }
    }

    private void subscribeToMqtt(String topic) {
        try {
            client.subscribe(topic, 0);
        } catch (MqttException e) {
            e.printStackTrace();
        }
    }

    /**
     * Manipulates the map once available.
     * This callback is triggered when the map is ready to be used.
     * This is where we can add markers or lines, add listeners or move the camera. In this case,
     * we just add a marker near Sydney, Australia.
     * If Google Play services is not installed on the device, the user will be prompted to install
     * it inside the SupportMapFragment. This method will only be triggered once the user has
     * installed Google Play services and returned to the app.
     */
    @Override
    public void onMapReady(GoogleMap googleMap) {
        mMap = googleMap;
        mMap.setMapType(4);
        builder = new LatLngBounds.Builder();
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
        } else {
            mMap.setMyLocationEnabled(true);
        }

        // Add a marker in Sydney and move the camera
        /*LatLng sydney = new LatLng(-34, 151);
        mMap.addMarker(new MarkerOptions().position(sydney).title("Marker in Sydney"));
        mMap.moveCamera(CameraUpdateFactory.newLatLngZoom(sydney,10.2f));*/
    }

    public void buttonClick(View view) {
        if(view.getId() == drZoom.getId()){
            centerOnPosition(dronePos);
        }else if(view.getId() == shwAtt.getId()){
            myDialog.setContentView(R.layout.attributes_popup);
            TextView txtclose =(TextView) myDialog.findViewById(R.id.txtclose);
            txtclose.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    myDialog.dismiss();
                }
            });
            myDialog.getWindow().setBackgroundDrawable(new ColorDrawable(Color.TRANSPARENT));
            myDialog.show();
        }
    }
}