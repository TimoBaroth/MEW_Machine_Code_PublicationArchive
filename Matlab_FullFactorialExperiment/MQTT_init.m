%%% TOPICS %%%% 


% Catridge Heating control 
keyset_cart = {'CartridgeTempOnOffIN','CartridgeTempOnOffOUT','CartridgeTempSET','CartridgeTempACT'};
valueset_cart = {'MEWRP4/CartridePrintHead/CONTROLIN_CartridgeTemperature','MEWRP4/CartridePrintHead/CONTROLOUT_CartridgeTemperature','MEWRP4/CartridePrintHead/SET_CartridgeTemperature','MEWRP4/CartridePrintHead/ACT_CartridgeTemperature'};
%CartridgeTempOnOffIN = 'MEWRP4/CartridePrintHead/CONTROLIN_CartridgeTemperature'; %send true/false to turn on/off
%CartridgeTempOnOffOUT = 'MEWRP4/CartridePrintHead/CONTROLOUT_CartridgeTemperature'; %returns status 
%CartridgeTempSET = 'MEWRP4/CartridePrintHead/SET_CartridgeTemperature'; %send setpoint temperature degC
%CartridgeTempACT = 'MEWRP4/CartridePrintHead/ACT_CartridgeTemperature'; %returns actual temperature 

%Ring Heating control  
keyset_ring = {'RingTempOnOffIN','RingTempOnOffOUT','RingTempSET','RingTempACT'};
valueset_ring = {'MEWRP4/CartridePrintHead/CONTROLIN_RingTemperature','MEWRP4/CartridePrintHead/CONTROLOUT_RingTemperature','MEWRP4/CartridePrintHead/SET_RingTemperature','MEWRP4/CartridePrintHead/ACT_RingTemperature'};
%RingTempOnOffIN = 'MEWRP4/CartridePrintHead/CONTROLIN_RingTemperature'; %send true/false to turn on/off
%RingTempOnOffOUT = 'MEWRP4/CartridePrintHead/CONTROLOUT_RingTemperature'; %returns status 
%RingTempSET = 'MEWRP4/CartridePrintHead/SET_RingTemperature'; %send setpoint temperature degC
%RingTempACT = 'MEWRP4/CartridePrintHead/ACT_RingTemperature'; %returns actual temperature 
 
%Pressure control
keyset_pressure = {'PressureOnOffIN','PressureOnOffOUT','PressureSET','PressureACT'};
valueset_pressure = {'MEWRP4/CartridePrintHead/CONTROLIN_Pressure','MEWRP4/CartridePrintHead/CONTROLOUT_Pressure','MEWRP4/CartridePrintHead/SET_Pressure','MEWRP4/CartridgePrintHead/ACT_Pressure'};
%PressureOnOffIN = 'MEWRP4/CartridePrintHead/CONTROLIN_Pressure'; %send true/false to turn on/off
%PressureOnOffOUT = 'MEWRP4/CartridePrintHead/CONTROLOUT_Pressure'; %returns status 
%PressureSET = 'MEWRP4/CartridePrintHead/SET_Pressure'; %send setpoint pressure in bar
%PressureACT = 'MEWRP4/CartridgePrintHead/ACT_Pressure'; %returns actual pressure 
 
%High voltage control 
keyset_hv = {'HVOnOffIN','HVOnOffOUT','HVSET','HVACT'};
valueset_hv = {'MEWRP4/HighVoltage/CONTROLIN_HVSupply','MEWRP4/HighVoltage/CONTROLOUT_HVSupply','MEWRP4/HighVoltage/SET_HVSupplyVoltage','MEWRP4/HighVoltage/ACT_HVSupplyVoltage'};
%HVOnOffIN = 'MEWRP4/HighVoltage/CONTROLIN_HVSupply'; %send true/false to turn on/off
%HVOnOffOUT = 'MEWRP4/HighVoltage/CONTROLOUT_HVSupply';  %returns status 
%HVSET = 'MEWRP4/HighVoltage/SET_HVSupplyVoltage'; %send setpoint pressure in kV
%HVACT = 'MEWRP4/HighVoltage/ACT_HVSupplyVoltage'; %returns actual voltage

%DIO/Diagnostics
keyset_dia = {'HVSupplyDoor','KinematicLimitSafety'};
valueset_dia = {'MEWRP4/DIO/HVPowerSupply','MEWRP4/DIO/LimitSwitchSafetyTriggered'};
%HVSupplyDoor = 'MEWRP4/DIO/HVPowerSupply';
%KinematicLimitSafety = 'MEWRP4/DIO/LimitSwitchSafetyTriggered';

%Experiement 
keyset_experiment = {'experiment_run_status','external_stop_signal'};
valueset_experiment = {'MATLAB/run','MATLAB/ext_stop'};

%speed
keyset_speed= {'set_speed'};
valueset_speed = {'CAXIS/speed_set'};

%experiment_run_status = 'MATLAB/run';


mqtt_keyset = [keyset_cart,keyset_ring,keyset_pressure,keyset_hv,keyset_dia,keyset_experiment,keyset_speed];
mqtt_valueset = [valueset_cart,valueset_ring,valueset_pressure,valueset_hv,valueset_dia,valueset_experiment,valueset_speed];
mqtt_topics = containers.Map(mqtt_keyset,mqtt_valueset);
clear keyset_cart keyset_ring keyset_hv keyset_dia keyset_experiment keyset_fluidnc
clear valueset_cart valueset_ring valueset_hv valueset_dia valueset_experiment valueset_fluidnc

%Computer vision - Jet 
%CVJetNozzleCollectorDistanceIN = 'MEWCV/MainCam/CONTROLIN_CVNozzleCollectorDistance'; %send true = fixed nozzle/collector distance, flase = dynamic deposition point measurement
%CVJetNozzleCollectorDistanceOUT = 'MEWCV/MainCam/CONTROLOUT_CVNozzleCollectorDistance'; %returns status
%CVJetMeasurementPointRelativeIN = 'MEWCV/MainCam/CONTROLIN_CVMeasurementPositionsRelative'; %send true = percentage, false = micrometers from nozzle
%CVJetMeasurementPointRelativeOUT = 'MEWCV/MainCam/CONTROLOUT_CVMeasurementPositionsRelative'; %returns status
%CVJetDiameterPositionsSET = 'MEWCV/MainCam/SET_JetDiametersPositions'; %send list with positions for diameter measurement
%CVJetAnglePositionsSET = 'MEWCV/MainCam/SET_JetAnglesPositions'; % send list with positions for angle measurement 
%CVJetSpinlineLengthPositionsSET = 'MEWCV/MainCam/SET_SpinlineLengthPositions'; %send list with positions for spinline length measurement  

%CVJetDiametersACT = 'MEWCV/MainCam/ACT_JetDiameters';
%CVJetAnglesACT = 'MEWCV/MainCam/ACT_JetAngles';
%CVJetSpinlineLengthACT = 'MEWCV/MainCam/SpinlineLength';
%CVJetAreaACT = 'MEWCV/MainCam/ACT_Area';
%CVJetNozzleCollectorDistanceACT = 'MEWCV/MainCam/ACT_NozzleCollectorDistance';

%%%% Client init %%%% 
mqttaddr = 'tcp://192.168.137.4'; %set mqtt broker adress and connection protocol
mqttport = 1883; %set port  
mqClient = mqttclient(mqttaddr, Port=mqttport); %create client 




