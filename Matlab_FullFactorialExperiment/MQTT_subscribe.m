%SCartridgeTempACT_SUB = subscribe(mqClient, CartridgeTempACT, QualityOfService=2, Callback=@CartridgeTempACT_callback);
%RingTempACT_SUB = subscribe(mqClient, RingTempACT, QualityOfService=2, Callback=@RingTempACT_callback);
%PressureACT_SUB = subscribe(mqClient, PressureACT, QualityOfService=2, Callback=@PressureACT_callback);
%HVACT_SUB = subscribe(mqClient, HVACT, QualityOfService=2, Callback=@HVACT_callback); 
%HVDOOR_SUB = subscribe(mqClient, mqtt_topics('HVSupplyDoor'), QualityOfService=2, Callback=@HVDOOR_callback); 


EXTERNALSTOP_SUB = subscribe(mqClient, mqtt_topics('external_stop_signal'), QualityOfService=2, Callback=@EXTERNALSTOP_callback); 

CARTTEMPACT_SUB = subscribe(mqClient, mqtt_topics('CartridgeTempACT'), QualityOfService=2); %dont need callback, we only read from time to time 
RINGTEMPACT_SUB = subscribe(mqClient, mqtt_topics('RingTempACT'), QualityOfService=2);

%global pressure_act %ugly but whatever
%global voltage_act
%global carttemp_act
%global ringtemp_act
%global hvdoor_act
global EXTERNAL_STOP


%function CartridgeTempACT_callback(topic,data)
%    global carttemp_act;
%    data2 = split(data,",");
%    carttemp_act = str2double(data2(2));
%end

%function RingTempACT_callback(topic,data)
%    global ringtemp_act;
%    data2 = split(data,",");
%    ringtemp_act = str2double(data2(2));
%end

%function PressureACT_callback(topic,data)
%    global pressure_act;
%    data2 = split(data,",");
%    pressure_act = str2double(data2(2));
%end

%function HVACT_callback(topic,data)
%    global voltage_act
%    data2 = split(data,",");
%    voltage_act = str2double(data2(2));
%end

%function HVDOOR_callback(topic,data)
%    global hvdoor_act
%    hvdoor_act = str2double(data);
%end


function EXTERNALSTOP_callback(topic,data)
    global EXTERNAL_STOP
    EXTERNAL_STOP = str2double(data);
end





%%%%% How to read values:

% single read: 
% readout = read(mqttcl, Topic=CartridgeHeatACT) 
% then e.g. last value read(end,:)
% => probs better for our needs

%or continuous read out with call backs and global variable:
%global temp 

%function CartridgeHeatACT_callback(topic,data)
%    global temp
%    temp = data;
%end
