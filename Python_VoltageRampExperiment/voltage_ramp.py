import numpy as np
import paho.mqtt.client as mqtt
import time

##### SET PARAMETERS HERE #####
initial_wait_time = 0  #### Wait time at start value
start = 3 ### start voltage
end = 6  ### end voltage
dT_dt = 0.2 # change per minute / ramp speed

##### Do not change code below #### 

mqttbroker = "192.168.137.4"
port = 1883

topic4 = "MEWRP4/HighVoltage/SET_HVSupplyVoltage"


client = mqtt.Client(protocol=mqtt.MQTTv5)  # create mqtt client, make sure to use v5 protocol 
client.connect(mqttbroker, port) #connect to broker using the defined adress and port
client.loop_start() #start the background loop to process messages 


sub_dt = 0.1 # update every 100 ms / 0.1 s 
sub_dT = (dT_dt/60)/(1/sub_dt) # delta temp per sub step

steps = [i for i in np.arange(start,end+sub_dT,sub_dT)]
#print(steps)

client.publish(topic4, start, qos=2)

if initial_wait_time != 0:
    print("initial wait time...")
    for i in range(0,initial_wait_time):
        print(initial_wait_time-i, 'minutes left')
        time.sleep(60)

print("starting parameter ramp..")

t = time.monotonic()
#print(t)
i = 0
while True:
    try:
        if time.monotonic() - t >= sub_dt :
            out = round(steps[i],2)
            client.publish(topic4, out, qos=2)
            #print("set:", round(steps[i],2))
            i += 1 
            t += sub_dt
    except: #fails if list end reached
        print("shut down..")
        exit()
       # client.publish(topic4, 0, qos=2) #turn off HV



    time.sleep(0.005) #reduce cpu load

