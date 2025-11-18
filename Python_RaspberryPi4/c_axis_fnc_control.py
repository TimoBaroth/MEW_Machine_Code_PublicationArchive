#import mqtt library
import paho.mqtt.client as mqtt
# serial comms
import serial 
#math 
import math
# OS functions etc
import time
import json
from threading import Lock, Event

### MQTT GLOBAL VARS ###
mqtt_dict = {}
mqttin_lock = Lock()
mqtt_envent = Event()

##################
### INIT MQTT  ###
##################

#MQTT connections config
mqttbroker = "192.168.137.4"
port = 1883
maintopic = "MEWRP4/CAxis_control" 
fnc_topic_speed_act = "FLUIDNC/speed_act"
fnc_topic_pos_act =  "FLUIDNC/position_act"


def msg_callback(client, userdata, message):
    global mqtt_dict, mqttin_lock, mqtt_event
    print("rec_mqtt")
    mqttin_lock.acquire() #get lock, so we don't overwrite while it is accessed 
    mqtt_dict = json.loads(message.payload)
    mqtt_envent.set()
    mqttin_lock.release()

### INIT MQTT COMMUNICATION ###
client = mqtt.Client(protocol=mqtt.MQTTv5)  # create mqtt client, make sure to use v5 protocol 
client.connect(mqttbroker, port) #connect to broker using the defined adress and port
client.loop_start() #start the background loop to process messages 
client.subscribe([(maintopic, 2)]) #subscribe to the topic with QOS2 
client.message_callback_add(maintopic, msg_callback) # add callback for specific sub-topic


####################
### INIT SERIAL  ###
####################

comportarduino = '/dev/ttyACM0'
arduino = serial.Serial(port=comportarduino, baudrate=115200, timeout=.1) 
print("arduino - connected to:", comportarduino)
time.sleep(2) #give the arduino some time to start up
comport0_fluidnc = '/dev/ttyUSB0'
comport1_fluidnc = '/dev/ttyUSB1'
try:
    fnc = serial.Serial(port=comport0_fluidnc, baudrate=115200, timeout=.1) 
    print("fludinc - connected to:", comport0_fluidnc)
except:
    print("failed to connectect to:", comport0_fluidnc)
    try:
        fnc= serial.Serial(port=comport1_fluidnc, baudrate=115200, timeout=.1) 
        print("fludinc - connected to:", comport1_fluidnc)
    except:
        print("failed to connectect to:", comport1_fluidnc)



##################
##################

class controller():
    def __init__(self):
        # stepper parameter
        self.c_axis_belt_ratio = 8.4 #mechanical ration belt drive
        self.c_axis_microsteps = 3200 # microsteps setting c-axis 
        #inputs_dict working copy 
        self.input_dict = {'c_axis_speed': 0}
        #fluidnc parameters
        self.fnc_spindle_speed = 0
        self.fnc_positions = 0
        #
        self.speed = 0
             

    def update_in_dict(self):
        if mqtt_envent.is_set():
            if mqttin_lock.acquire(timeout=0.01): #get lock, so we don't overwrite while it is accessed , wait max 10 ms
                for key in mqtt_dict: #copy values in our working dict
                    print(mqtt_dict)
                    if key in self.input_dict:
                        self.input_dict[key] = mqtt_dict[key]
                mqtt_envent.clear()
                mqttin_lock.release()
        
    def calc_CAxis_sps(self, coll_radius, speed):
        collector_circumference = coll_radius * 2 * math.pi 
        rpm = speed / collector_circumference #rotations per minute 
        steps_per_sec_c_axis = rpm * self.c_axis_belt_ratio * self.c_axis_microsteps * (1/60) #steps per second
        return steps_per_sec_c_axis
        
    def getFluidNC(self):
        try:
            fnc.write(b"?")
            status_msg = str(fnc.read_until(b">"))
            self.fnc_spindle_speed = status_msg.split("FS:")[1].split("|")[0]
            positions = status_msg.split("MPos:")[1].split("|")[0].split(",")
            self.fnc_positions = ",".join(positions)
            #print(positions)
            #print(spindle_speed)
            #print()
        except:
            print("FluidNC data aquisition failed")
            self.fnc_spindle_speed = "-1, -1"
            self.fnc_positions = "-1, -1, -1, -1, -1, -1"


    def run(self):
        last_c_sps = 0
        while True:
            self.update_in_dict() #update inputs 
        
            self.speed = self.input_dict['c_axis_speed']
            c_sps = self.calc_CAxis_sps(self.input_dict['eff_collector_radius'], self.speed )
            if last_c_sps != c_sps :
                last_c_sps = c_sps
                ao = {"c_axis_speed": round(c_sps,2)}
                arduino.write(bytes(json.dumps(ao), "utf-8"))  #
                client.publish("CAXIS/speed_set", json.dumps(self.speed ), qos=2)
            self.getFluidNC()
            client.publish(fnc_topic_speed_act, str(self.fnc_spindle_speed), qos=2)
            client.publish(fnc_topic_pos_act, str(self.fnc_positions), qos=2)

            time.sleep(0.005)
            
            

c = controller()
while True:
    c.run()