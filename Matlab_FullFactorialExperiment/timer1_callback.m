function timer1_callback(obj, event, full_factorial_lvl, one_temp, number_combinations, t1, t2, mqClient, mqtt_topics, t1_period)
    global next_combi
    %global effective_collector_circumference
    global steps_speed_factor 
    persistent last_ring_temp
    persistent last_cart_temp
    persistent last_voltage
    persistent last_speed
    persistent last_pressure
    persistent wait_mode 
    persistent last_set_time
    persistent sensorValuesRING
    persistent sensorValuesCART
    persistent idxRING
    persistent idxCART


    %disp("timer function called")



    % Read cart/ring temp
    readout = read(mqClient, Topic=mqtt_topics('CartridgeTempACT'));
    readout = split(readout.Data(end,:),','); %only last value
    act_cart_temp_sample = str2double(readout(2)); %only temp value
    readout = read(mqClient, Topic=mqtt_topics('RingTempACT'));
    readout = split(readout.Data(end,:),','); %only last value
    act_ring_temp_sample = str2double(readout(2)); %only temp value


    movavgwindow = 30;
    %init persistent variables, upon first function call 
    if isempty(wait_mode)
        wait_mode = -1;
        last_set_time = datetime('now', 'timezone', 'utc');
        sensorValuesRING = ones(1,movavgwindow)*act_ring_temp_sample;
        sensorValuesCART = ones(1,movavgwindow)*act_cart_temp_sample;
        idxRING = 1;
        idxCART = 1;
        %disp("wait mode init")
    end
   


    function [result,new_idx,new_arr] = movingAverageCalculation(arr, value, idx)
            arr(idx) = value;
            idx = idx + 1;
            if(idx > movavgwindow)
                idx = 1;
            end
            result = sum(arr)/length(arr);
            new_idx = idx;
            new_arr = arr;
    end


    % calc moving average of temp, to avoid waiting for temp due to
    % measurement noise
    [act_cart_temp, idxCART, sensorValuesCART] = movingAverageCalculation(sensorValuesCART, act_cart_temp_sample, idxCART);
    [act_ring_temp, idxRING, sensorValuesRING] = movingAverageCalculation(sensorValuesRING, act_ring_temp_sample, idxRING);
        
    %if initialised, do normal check
    if not(wait_mode == -1)
        %disp("normal check")
        
        % if we have a difference of more than 1C from setpoint, we have
        % not reached equilibrium, so:
        % go to fast mode, by checking again every 10 seconds,
        % continue with normal pace once we reached setpoint
        % this makes sure we wait for temperature to reach setpoint before
        % going to next experimental parameter combination
        if (abs(act_ring_temp-last_ring_temp) > 1.5 | abs(act_cart_temp-last_cart_temp) > 1.5) & wait_mode == 0
            wait_mode = 1;
            disp("Waiting for temp. equilibrium")
            return %return without setting next parameter combi
        elseif (abs(act_ring_temp-last_ring_temp) > 1.5 | abs(act_cart_temp-last_cart_temp) > 1.5) & wait_mode == 1
            return %return without setting next parameter combi
        elseif wait_mode == 1
            wait_mode = 0;
            last_set_time = datetime('now', 'timezone', 'utc'); %update time reference
            disp("temp equilibrium reached, continuing...")
        end 
    end
    



    % if everything is ok, set next parameter combination
    time = datetime('now', 'timezone', 'utc');
    if ((abs(minutes(time - last_set_time)) >= t1_period) | wait_mode == -1) & next_combi <= number_combinations
        %disp("time passed or first run")
        if wait_mode == -1 %so we set first parameter set regardless of time
            wait_mode = 0; 
        end

        voltage = full_factorial_lvl(next_combi,3);
        speed = full_factorial_lvl(next_combi,1);
        pressure = full_factorial_lvl(next_combi,2);
        %if only one temp, set both to same 
        if one_temp
            cart_temp = full_factorial_lvl(next_combi,4);
            ring_temp = cart_temp;
        else
            cart_temp = full_factorial_lvl(next_combi,4);
            ring_temp = full_factorial_lvl(next_combi,5);
        end
        
        disp(' ')
        disp("Setting combination")
        %disp("Last combination: No." + (next_combi-1))
        %disp("Voltage [kV]: " + last_voltage + ", Speed [mm/min]: " + last_speed + ", Pressure [bar]: " + last_pressure + ", Cartridge Temp [C]: " + last_cart_temp +  ", Ring Temp [C]: " + last_ring_temp)
        disp("New combination: No." + next_combi + "/" + number_combinations)
        disp("Voltage [kV]: " + voltage + ", Speed [mm/min]: " + speed + ", Pressure [bar]: " + pressure + ", Cartridge Temp [C]: " + cart_temp +  ", Ring Temp [C]: " + ring_temp)
    
        % if everything is ok, and time expired=> set next parameter combination
    
       
    
        %write to MQTT
        % cartridge temp
        write(mqClient, mqtt_topics('CartridgeTempSET'), string(cart_temp));
        % ring temp
        write(mqClient, mqtt_topics('RingTempSET'), string(ring_temp));
        %speed    
        write(mqClient, mqtt_topics('set_speed'), string(speed));
        %pressure
        write(mqClient, mqtt_topics('PressureSET'), string(pressure));
        % voltage
        write(mqClient, mqtt_topics('HVSET'), string(voltage));
        
        %save last values
        last_cart_temp = cart_temp;
        last_ring_temp = ring_temp;
        last_speed = speed;
        last_pressure = pressure;
        last_voltage =  voltage;
        last_set_time = datetime('now', 'timezone', 'utc'); % save when we last changes the parameters
      
        next_combi = next_combi + 1; %iterate to next combination
   

    
  elseif (abs(minutes(time - last_set_time)) >= t1_period) & next_combi > number_combinations
        write(mqClient, mqtt_topics('experiment_run_status'), "false"); %stop data recoding 
        write(mqClient, mqtt_topics('RingTempOnOffIN'), "false");
        write(mqClient, mqtt_topics('CartridgeTempOnOffIN'), "false");
        write(mqClient, mqtt_topics('PressureOnOffIN'), "false");
        write(mqClient, mqtt_topics('HVOnOffIN'), "false");
        %write(mqClient, mqtt_topics('fluidnc_set_speed'), string(0));
    write(mqClient, mqtt_topics('set_speed'), strig(0));
        disp("Run completed");
        clear last_ring_temp last_cart_temp last_voltage last_speed last_pressure wait_mode last_set_time
        stop(t1); %stop timer
        stop(t2)
    
    end
end

%write(mqClient, mqtt_topics('fluidnc_set_speed'), string(full_factorial_lvl(next_combi,2)));

% old old 
%old speed set is now handled by external python function
%speed_mm_min = full_factorial_lvl(next_combi,2); % get next speed 
%speed_deg_min = (speed_mm_min*360)/effective_collector_circumference;
%degrees = speed_deg_min*(t1.Period/60); %calc the distance in degrees to travel in next time step 
%msgpart = "G91 G1 C-" + degrees + " F" + speed_deg_min;
%msg = [unicode2native(msgpart) newline];
%write(telnetclient, msg);