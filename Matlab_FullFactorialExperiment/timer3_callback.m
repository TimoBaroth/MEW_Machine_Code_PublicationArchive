function timer3_callback(obj, event, t3, telnetclient, speed_mm_min)
    global effective_collector_circumference
    
    speed_deg_min = (speed_mm_min*360)/effective_collector_circumference; %calc speed in degrees/minute
    degrees = speed_deg_min*(t3.Period/60); %calc the distance in degrees to travel in next time step 
    msgpart = "G91 G1 C-" + degrees + " F" + speed_deg_min; %prepate the message
    msg = [unicode2native(msgpart) newline]; %prepare second step
    write(telnetclient, msg); %send message to fluidnc
end