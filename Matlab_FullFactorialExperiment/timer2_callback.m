function timer2_callback(obj, event, t1, t2, mqClient, mqtt_topics)
    global EXTERNAL_STOP
    global next_combi
    
    if EXTERNAL_STOP == 1 % if door is opened during test run, stop as we can't experiment without HV
        disp("Error - EXTERNAL STOP" + newline + " Combination: " + string(next_combi-1));
        %write(mqClient, mqtt_topics('fluidnc_set_speed'), string(0));
        write(mqClient, mqtt_topics('set_speed'), string(0));
        write(mqClient, mqtt_topics('experiment_run_status'), "false"); %stop data recoding 
        write(mqClient, mqtt_topics('RingTempOnOffIN'), "false");
        write(mqClient, mqtt_topics('CartridgeTempOnOffIN'), "false");
        write(mqClient, mqtt_topics('PressureOnOffIN'), "false");
        write(mqClient, mqtt_topics('HVOnOffIN'), "false");
        stop(t1); %stop timer
        stop(t2);
    end
end