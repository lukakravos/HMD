#!/usr/bin/env python3



####Defining needed Imports

import time
import threading
import paho.mqtt.client as mqtt
from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil

####Defining needed MQTT-Topics to publish to

posTop = 'htl/diplomarbeit/wildschutzdrohne/drone/position/latLng'
headTop = 'htl/diplomarbeit/wildschutzdrohne/drone/heading'
altTop = 'htl/diplomarbeit/wildschutzdrohne/drone/altitude'
firmVerTop = 'htl/diplomarbeit/wildschutzdrohne/drone/version'
gpsTop = 'htl/diplomarbeit/wildschutzdrohne/drone/gps_state'
speedTop = 'htl/diplomarbeit/wildschutzdrohne/drone/speed'
batTop = 'htl/diplomarbeit/wildschutzdrohne/drone/battery'
modeTop = 'htl/diplomarbeit/wildschutzdrohne/drone/mode'
armdTop = 'htl/diplomarbeit/wildschutzdrohne/drone/isArmed'
vehStatTop = 'htl/diplomarbeit/wildschutzdrohne/drone/vehicleState'



####Connecting to FC with Dronekit and printing FC-Firmware-Version

print('Connecting...')
vehicle = connect('udp:192.168.7.168:14551')

#vehicle.wait_ready('autopilot version')
#print("Autopilot version: %s"%vehicle.version)
print("Connected!")



####Setting up required MQTT-Methods and settings and connecting to MQTT

def on_connect(client, userdata, flags, rc):
	client.subscribe('posTop');
	client.subscribe('headTop');
	client.subscribe('altTop');
	client.subscribe('firmVerTop');
	client.subscribe('gpsTop');
	client.subscribe('speedTop');
	client.subscribe('batTop');
	client.subscribe('modeTop');
	client.subscribe('armdTop');
	client.subscribe('vehStatTop');

def on_publish(client,userdata,result):
	#print("data published \n")
	pass

BROKER_ADDRESS = "iotmqtt.htl-klu.at"

client = mqtt.Client()
client.on_connect = on_connect
client.on_publish = on_publish

client.username_pw_set(username="htl-IoT", password="iot..2015")

client.connect(BROKER_ADDRESS)

print("Connected to MQTT Broker: " + BROKER_ADDRESS)



####Clearing existing Waypoints and adding new Waypoints

#cmds = vehicle.commands;

#cmds.clear();

#cmd1=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10)
#cmds.add(cmd1)

#cmds.upload()



####Defining Attributes and getting current Time in seconds

lat = 0
lng = 0
alt = 0
hed = 0
bat = ""
firmVer = str(vehicle.version)
armd = vehicle.armed
mode = vehicle.mode.name
speed = 0
vehStat = vehicle.system_status.state
gps = ""
oldLat = 0;
oldLng = 0;



####Creating Callback functions for Attributes

def location_callback(self, attr_name, value):
	global lat
	global lng
	global alt
	lat = value.lat
	lng = value.lon
	alt = value.alt
	pos = str(lat) + ', ' + str(lng);
	global oldLat
	global oldLng
	if lat != oldLat or lng != oldLng:
		oldLat = lat;
		oldLng = lng;
		client.publish(posTop, pos, qos=0, retain=True);
	client.publish(altTop, value.alt);

def head_callback(self, attr_name, value):
	global hed
	hed = value;
	client.publish(headTop, hed)

def bat_callback(self, attr_name, value):
	global bat
	bat = str(value.voltage)+', '+str(value.current)+', '+str(value.level);
	client.publish(batTop, bat)

def armd_callback(self, attr_name, value):
	global armd
	armd = value

def mode_callback(self, attr_name, value):
	global mode
	mode = value.name
	#client.publish(modeTop, mode)

def speed_callback(self, attr_name, value):
	global speed
	speed = value
	client.publish(speedTop, speed)

def vehStat_callback(self, attr_name, value):
	global vehStat
	vehStat = value;

def gps_callback(self, attr_name, value):
	global gps
	gps = value.satellites_visible;

def firmVer_callback(self, attr_name, value):
	global firmVer
	firmVer = value



####Adding needed attribute listeners
vehicle.add_attribute_listener('armed', armd_callback)
vehicle.add_attribute_listener('location.global_relative_frame', location_callback)
vehicle.add_attribute_listener('battery', bat_callback)
vehicle.add_attribute_listener('system_status.state', vehStat_callback)
vehicle.add_attribute_listener('gps_0', gps_callback)
vehicle.add_attribute_listener('heading', head_callback)
vehicle.add_attribute_listener('mode', mode_callback)
vehicle.add_attribute_listener('version', firmVer_callback)



####Running every 0.5 second
added = False
def pubAttr():
	threading.Timer(0.5, pubAttr).start()
	print(armd)
	client.publish(armdTop, armd);
	client.publish(modeTop, mode);
	client.publish(vehStatTop, vehStat);
	client.publish(gpsTop, gps);
	client.publish(firmVerTop, firmVer)
	global added
	if armd:
		if not added:
			added = True
			vehicle.add_attribute_listener('groundspeed', speed_callback)
		client.publish(speedTop, speed);
	else:
		if added:
			added = False
			vehicle.remove_attribute_listener('groundspeed', speed_callback)
			speed = 0;

pubAttr()



####Taking off and starting Mission
while not vehicle.commands:
    print "Waiting for Routeplaning"
    time.sleep(1)
    
while not vehicle.is_armable:
    print "Waiting for vehicle to initialise..."
    time.sleep(1)

print "Arming motors"
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed  = True

while not vehicle.armed:
    print "Waiting for arming..."
    time.sleep(1)

print "Taking off!"
vehicle.simple_takeoff(aTargetAltitude)

print "Starting Mission"
vehicle.mode = VehicleMode("AUTO")



#####Running endless loop
while True:
	print("j")
