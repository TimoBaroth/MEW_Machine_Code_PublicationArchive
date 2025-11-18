#import mqtt library
import paho.mqtt.client as mqtt
# gpio libarys
import gpiozero
import daqhats
import daqhats_utils
# Circuitpython libs 
import board                                            
import busio
import adafruit_mcp4728
# PID libary
import simple_pid
# OS functions etc
import time 
import datetime
import os 
import sys
import errno

#########################
### CLASS DEFINITIONS ###
######################### 
#implementing temp control in class, to reduce chance of mistakes in main loop
class temp_controller_mcc_AD8495: #PID temp controller with interface for MCC ADC board and AD8495 TC board
    def __init__(self, heater, mcchat, mccchannel, mccoption, parameters):
        ### Min/Max/Limit Variables ###
        self.T_abs_max = parameters.get('T_abs_max')         # upper temp limit for safety shut off (e.g. detect broken wire)
        self.T_abs_min = parameters.get('T_abs_min')         # lower temp limit for safety shut off (e.g. detect broken wire)
        self.T_dev_up_max = parameters.get('T_dev_up_max')      # max deviation above set point for safety shut off (e.g. overshoot/runaway)
        self.T_dev_time_max = parameters.get('T_dev_time_max')    # max deviation from set point for time out 
        self.time_out_temp = parameters.get('time_out_temp')     # time intervall for check of |(T_act-Tset)| > T_dev_time_max 
        self.max_heating_power = parameters.get('max_heating_power') # maximum relative heating power 0...1
        ### PID variables ###
        self.pid_P = parameters.get('pid_P')   # PID proportional 
        self.pid_I = parameters.get('pid_I')   # PID integral 
        self.pid_D = parameters.get('pid_D')   # PID deviation
        ### PID ###
        self.pid = simple_pid.PID(self.pid_P, self.pid_I, self.pid_D, output_limits=(0.0,self.max_heating_power), auto_mode=False)
        ### I/O ###
        self.heater = heater
        self.mcchat = mcchat
        self.mccchannel = mccchannel
        self.mccoption = mccoption
        ### some more variables/flags ###
        self.control_active = False
        self.T_controller_error = [] # error list 
        self.heater_power = 0 #rel. heater power 0...1
        self.T_read = 0 #value from sensor 
        self.T_set = 0
        self.time_temp_check = 0 #time for deviation check 
        self.current_T_dev_time_check = False #setup flag 
        self.last_T_dev_time_check = False   #setup flag 
        
        
        
    def is_number(self, n): #function to check if n is a (float) number, important to prevent program crash if there is a wrong input
        return isinstance(n, (int,float))
        #try:
        #    float(n)
        #    return True
        #except ValueError:
        #    return False

    def read_temperature(self):
        voltage_reading = self.mcchat.a_in_read(self.mccchannel, self.mccoption) #read temperature from sensor 
        #convert from voltage reading to temperature 
        # AD8495 (adafruit 1778): Temp[C] = (Vout-1.25) / 0.005 
        #print("Temperature Reading: ", self.T_read) #old
        #print(self.T_read) #debug only
        if self.is_number(voltage_reading) == True:
            self.T_read = (voltage_reading-1.25) / 0.005 #see conversion comment above
            #print(self.T_read)
            if self.T_read >= self.T_abs_max or self.T_read <= self.T_abs_min:  #check for absolute max upper/lower temperatur, e.g. broken wire 
                self.T_controller_error.append("SensorMinMaxError") #set error
        else:
            self.T_controller_error.append("SensorReadingNaN") #set error 

    # function to safely set new T_set value, checks for valid type (int or float) using is_number function
    # tested
    def update_T_set(self, n):
        if self.is_number(n) == True: #check if n is a number, if so..
            self.T_set = n #store new set point
            self.pid.setpoint = self.T_set #set new PID setpoint
            
        else: 
            self.T_controller_error.append("T_setNaN") #otherwise, set error 
        
    def set_heater_power(self,n):
        if self.is_number(n) == True: #check if n is a number
            if n > self.max_heating_power or n < 0.0: #check if in allowed range 0.0 ... 1.0, if not..
                self.heater_off()
                self.T_controller_error.append("HeaterPowerOutOfRange") #set error 
            else:
                self.heater_power = n #store new rel. heater power
                self.heater.value = self.heater_power #turn heater to requested value 
        else: # if n is not a number
            self.heater_off()
            self.T_controller_error.append("HeaterPowerNaN") #set error 


    def heater_off(self):
            self.heater_power = 0 #set heating power to 0
            self.heater.value = self.heater_power #turn off heater          

      #  if self.control_active == False and self.T_set > 0: #check if control is not active and set point is equal/lower zero, allow control to reset/startup
      #      self.control_active = True #turn control back on
      #      self.T_controller_error = "Ok" #reset error 
      #      #self.pid.auto_mode = True #activate pid controller 
      #      #self.time_temp_check = time.time() #get current time for time based checks
      #  elif self.T_set <= 0: #if T_set is 
      #      self.control_active = False
      #      self.pid.auto_mode = False #de-activate pid controller 

    #call this function to run control loop, will always return sensor reading, rel. heater power, control_active flag and error list
    #if control is set active it will also run PID loop and set heater accordingly
    def run(self): 
        #self.update_T_set(T_set) #update T_set, not needed for MQTT implementation
        self.read_temperature() #read current temperature from sensor
        #there might be errors during this, so check if we can continue 
        if len(self.T_controller_error) != 0: #if there is something in the error list 
            self.heater_off() #call function, turns off heater/sets heater power accordingly
            self.control_active = False # set flag that control is not active
        
        #if there are no errors we continue here:
        #check if control_active flag is true 
        elif self.control_active == True:
            self.limit_check() #check for temp limit conditions 
            if len(self.T_controller_error) != 0: #if there is something in the error list  
                self.heater_off() #call function, turns off heater/sets heater power accordingly
                self.control_active = False # set flag that control is not active
              
            else: #if no limit errors, continue:
                self.heater_power = self.pid(self.T_read) #calculate new PID heater power 
                self.set_heater_power(self.heater_power) #try to set new power, will catch errors 
                if len(self.T_controller_error) != 0: #if there is something in the error list  
                    self.heater_off() #call function, turns off heater/sets heater power accordingly
                    self.control_active = False # set flag that control is not active

            
        err = self.T_controller_error.copy() #copy our errors must be a copy otherwise err will point to original list and be cleared too
        self.T_controller_error.clear() #clear error list for next run 
        return [self.T_read, self.heater_power, self.control_active, err]    #return list of sensor reading, rel. heater power and errors

    #call this function to activate/deactivate the control 
    def mode(self, m):
        if isinstance(m, str): #check that is str and turn into bool
            if m == "True":
                self.pid.auto_mode = False
                self.pid.auto_mode = True #toggle pid to start with fresh output/states
                self.time_temp_check = time.time() #update time for next comparison of limits check 
                self.control_active = True
            elif m == "False":
                self.heater_off()
                self.pid.auto_mode = False
                self.control_active = False
                



    def limit_check(self):   
        # CALL AFTER read_temperature error check, otherwise self.T_read might NaN error might happen
        # check for overshoot/runaway, does not make sens, will get triggerd by reduction of set point, time vs deviation check should be enough
        #if (self.T_read-self.T_set) >= self.T_dev_up_max:
         #   self.T_controller_error.append("OvershootError") # set error flag
            #print("first check tripped")  # debug only

        
        # check for deviation time out -> delta between T_act and T_set greater T_dev_time_max for time_out_temp minutes    
        if time.time()-self.time_temp_check >= self.time_out_temp*60:
            self.time_temp_check = time.time() #update time for next comparison 
            #print("second check run") # debug only
            #check current deviation 
            if abs(self.T_read-self.T_set) >= self.T_dev_time_max:
                self.current_T_dev_time_check = True
            else:
                self.current_T_dev_time_check = False
            # check if deviation is too big at current time/sample and if it was at the last time/sample, if so set error
            if  self.current_T_dev_time_check == True and self.last_T_dev_time_check == True:
                self.T_controller_error.append("T_setT_readDeviationTimeOut") 
                #print("second check tripped")  #debug only
                #print(self.T_controller_error) #debug only
            # update flag for next check 
            self.last_T_dev_time_check = self.current_T_dev_time_check

class pressure_controller_mcc_MCP4728: #PID temp controller with interface for MCC ADC board and AD8495 TC board
    def __init__(self, DAC, mcchat, mccchannel, mccoption, parameters):
        ### Min/Max/Limit Variables ###
        self.P_abs_max = parameters.get('P_abs_max')         # upper temp limit for safety shut off (e.g. detect broken wire)
        self.P_abs_min = parameters.get('P_abs_min')         # lower temp limit for safety shut off (e.g. detect broken wire)
        self.P_dev_up_max = parameters.get('P_dev_up_max')      # max deviation above set point for safety shut off (e.g. overshoot/runaway)
        self.P_dev_time_max = parameters.get('P_dev_time_max')    # max deviation from set point for time out 
        self.time_out = parameters.get('time_out')     # time intervall for check of |(T_act-Tset)| > T_dev_time_max 
        self.max_output = parameters.get('max_output') # maximum relative heating power 0...1
        ### PID variables ###
        self.pid_P = parameters.get('pid_P')   # PID proportional 
        self.pid_I = parameters.get('pid_I')   # PID integral 
        self.pid_D = parameters.get('pid_D')   # PID deviation
        ### PID ###
        self.pid = simple_pid.PID(self.pid_P, self.pid_I, self.pid_D, output_limits=(0.0,self.max_output), auto_mode=False)
        ### I/O ### 
        self.DAC = DAC
        self.mcchat = mcchat
        self.mccchannel = mccchannel
        self.mccoption = mccoption
        ### some more variables/flags ###
        self.control_active = False
        self.P_controller_error = [] # error list 
        self.output = 0 #output 
        self.P_read = 0 #value from sensor 
        self.P_read_wooffset = 0 
        self.P_set = 0
        self.time_check = 0 #time for deviation check 
        self.current_P_dev_time_check = False #setup flag 
        self.last_P_dev_time_check = False   #setup flag 
        # Pressure sensor offset 
        self.offset = 0
        
    
    def update_offset(self,n):
        self.offset = n
        
    def is_number(self, n): #function to check if n is a (float) number, important to prevent program crash if there is a wrong input
        return isinstance(n, (int,float))
        #try:
        #    float(n)
        #    return True
        #except ValueError:
        #    return False

    def read_pressure(self):
        voltage_reading = self.mcchat.a_in_read(self.mccchannel, self.mccoption) #read temperature from sensor 
        #convert from voltage reading to pressure 
        # Schmalz VS P10 SA M8-4 translates 0-10 Bar directly to 0-10V 
        if self.is_number(voltage_reading) == True:
            self.P_read_wooffset = voltage_reading 
            self.P_read = voltage_reading-self.offset #see conversion comment above
            if self.P_read >= self.P_abs_max or self.P_read <= self.P_abs_min:  #check for absolute max upper/lower temperatur, e.g. broken wire 
                self.P_controller_error.append("SensorMinMaxError") #set error
        else:
            self.P_controller_error.append("SensorReadingNaN") #set error 

    # function to safely set new T_set value, checks for valid type (int or float) using is_number function
    # tested
    def update_P_set(self, n):
        if self.is_number(n) == True: #check if n is a number, if so..
            self.P_set = n #store new set point
            self.pid.setpoint = self.P_set #set new PID setpoint
            
        else: 
            self.P_controller_error.append("P_setNaN") #otherwise, set error 
        
    def set_output(self,n):
        if self.is_number(n) == True: #check if n is a number
            if n > self.max_output or n < 0.0: #check if in allowed range if not..
                self.output_off()
                self.P_controller_error.append("OutputOutOfRange") #set error 
            else:
                self.output = self.P_set + n # have to add P_set as pilot control / vorsteuerung
                dac_output = int(self.output*(4095/10)) #scale 0...10 to 0...4095 
                #print(dac_output)
                if dac_output > 4095:
                    dac_output = 4095 #limit to max valid value
                self.DAC.channel_a.raw_value = dac_output #turn DAC to requested value
        else: # if n is not a number
            self.output_off()
            self.P_controller_error.append("OutputNaN") #set error 


    def output_off(self):
            self.output = 0 #set output to 0
            self.DAC.channel_a.raw_value = self.output #turn off      

      #  if self.control_active == False and self.T_set > 0: #check if control is not active and set point is equal/lower zero, allow control to reset/startup
      #      self.control_active = True #turn control back on
      #      self.T_controller_error = "Ok" #reset error 
      #      #self.pid.auto_mode = True #activate pid controller 
      #      #self.time_temp_check = time.time() #get current time for time based checks
      #  elif self.T_set <= 0: #if T_set is 
      #      self.control_active = False
      #      self.pid.auto_mode = False #de-activate pid controller 

    #call this function to run control loop, will always return sensor reading, rel. heater power, control_active flag and error list
    #if control is set active it will also run PID loop and set heater accordingly
    def run(self): 
        #self.update_T_set(T_set) #update T_set, not needed for MQTT implementation
        self.read_pressure() #read current temperature from sensor
        #print(self.P_read)
        #there might be errors during this, so check if we can continue 
        if len(self.P_controller_error) != 0: #if there is something in the error list 
            self.output_off() #call function, turns off heater/sets heater power accordingly
            self.control_active = False # set flag that control is not active
        
        #if there are no errors we continue here:
        #check if control_active flag is true 
        elif self.control_active == True:
            self.limit_check() #check for temp limit conditions 
            if len(self.P_controller_error) != 0: #if there is something in the error list  
                self.output_off() #call function, turns off heater/sets heater power accordingly
                self.control_active = False # set flag that control is not active
              
            else: #if no limit errors, continue:
                #self.output = self.pid(self.P_read) #calculate new PID heater power 
                self.set_output(self.pid(self.P_read)) #try to set new power, will catch errors 
                if len(self.P_controller_error) != 0: #if there is something in the error list  
                    self.output_off() #call function, turns off heater/sets heater power accordingly
                    self.control_active = False # set flag that control is not active

            
        err = self.P_controller_error.copy() #copy our errors must be a copy otherwise err will point to original list and be cleared too
        self.P_controller_error.clear() #clear error list for next run 
        return [self.P_read, self.output, self.control_active, err]    #return list of sensor reading, rel. heater power and errors

    #call this function to activate/deactivate the control 
    def mode(self, m):
        if isinstance(m, str): #check that is str and turn into bool
            if m == "True":
                self.pid.auto_mode = False
                self.pid.auto_mode = True #toggle pid to start with fresh output/states
                self.time_check = time.time() #update time for next comparison of limits check 
                self.control_active = True
            elif m == "False":
                self.output_off()
                self.pid.auto_mode = False
                self.control_active = False
                



    def limit_check(self):   
        # CALL AFTER read_temperature error check, otherwise self.T_read might NaN error might happen
        # check for overshoot/runaway, does not make sens, will get triggerd by reduction of set point, time vs deviation check should be enough
        #if (self.T_read-self.T_set) >= self.T_dev_up_max:
         #   self.T_controller_error.append("OvershootError") # set error flag
            #print("first check tripped")  # debug only

        
        # check for deviation time out -> delta between T_act and T_set greater T_dev_time_max for time_out_temp minutes    
        if time.time()-self.time_check >= self.time_out*60:
            self.time_check = time.time() #update time for next comparison 
            #print("second check run") # debug only
            #check current deviation 
            if abs(self.P_read-self.P_set) >= self.P_dev_time_max:
                self.current_P_dev_time_check = True
            else:
                self.current_P_dev_time_check = False
            # check if deviation is too big at current time/sample and if it was at the last time/sample, if so set error
            if  self.current_P_dev_time_check == True and self.last_P_dev_time_check == True:
                self.P_controller_error.append("P_setP_readDeviationTimeOut") 
                #print("second check tripped")  #debug only
                #print(self.T_controller_error) #debug only
            # update flag for next check 
            self.last_P_dev_time_check = self.current_P_dev_time_check

class voltage_controller_mcc_MCP4728: #PID temp controller with interface for MCC ADC board and AD8495 TC board
    def __init__(self, DAC, mcchat, mccchannel1, mccchannel2, mccoption, parameters):
        ### Min/Max/Limit Variables ###
        self.abs_max = parameters.get('abs_max')         # upper temp limit for safety shut off (e.g. detect broken wire)
        self.abs_min = parameters.get('abs_min')         # lower temp limit for safety shut off (e.g. detect broken wire)
        self.dev_up_max = parameters.get('dev_up_max')      # max deviation above set point for safety shut off (e.g. overshoot/runaway)
        self.dev_time_max = parameters.get('dev_time_max')    # max deviation from set point for time out 
        self.time_out = parameters.get('time_out')     # time intervall for check of |(T_act-Tset)| > T_dev_time_max 
        self.max_output = parameters.get('max_output') # maximum relative heating power 0...1
        ### PID variables ###
        self.pid_P = parameters.get('pid_P')   # PID proportional 
        self.pid_I = parameters.get('pid_I')   # PID integral 
        self.pid_D = parameters.get('pid_D')   # PID deviation
        ### PID ###
        self.pid = simple_pid.PID(self.pid_P, self.pid_I, self.pid_D, output_limits=(0.0,self.max_output), auto_mode=False)
        ### I/O ### 
        self.DAC = DAC
        self.mcchat = mcchat
        self.mccchannel1 = mccchannel1
        self.mccchannel2 = mccchannel2
        self.mccoption = mccoption
        ### some more variables/flags ###
        self.control_active = False
        self.controller_error = [] # error list 
        self.output = 0 #output 
        self.voltage_read = 0 
        self.current_read = 0
        self.set = 0
        self.time_check = 0 #time for deviation check 
        self.current_dev_time_check = False #setup flag 
        self.last_dev_time_check = False   #setup flag 
        
        
        
    def is_number(self, n): #function to check if n is a (float) number, important to prevent program crash if there is a wrong input
        return isinstance(n, (int,float))
        #try:
        #    float(n)
        #    return True
        #except ValueError:
        #    return False

    def read(self):
        voltage_reading = self.mcchat.a_in_read(self.mccchannel1, self.mccoption) #read temperature from sensor 
        #print("raw volt reading:")
        #print(voltage_reading)
        #convert from voltage reading to HV reading
        # Gamma RC3-20P/24/M1219 translates 0-20 kV to 0-2V 
        if self.is_number(voltage_reading) == True:
            self.voltage_read = voltage_reading*10 #see conversion comment above
            #print(self.voltage_read) 
            if self.voltage_read >= self.abs_max or self.voltage_read <= self.abs_min:  #check for absolute max upper/lower temperatur, e.g. broken wire 
                self.controller_error.append("VoltageMinMaxError") #set error
        else:
            self.controller_error.append("VoltageReadingNaN") #set error

        current_reading = self.mcchat.a_in_read(self.mccchannel2, self.mccoption) #read temperature from sensor 
        #convert from voltage reading to HV reading
        # Gamma RC3-20P/24/M1219 translates 0-150uA to 0-1.5V 
        if self.is_number(current_reading) == True:
            self.current_read = current_reading*100 #output in uA see conversion comment above
            #print(self.current_read)
        else:
            self.controller_error.append("CurrentReadingNaN") #set error 


    # function to safely set new T_set value, checks for valid type (int or float) using is_number function
    # tested
    def update_set(self, n):
        if self.is_number(n) == True: #check if n is a number, if so..
            self.set = n #store new set point
            self.pid.setpoint = self.set #set new PID setpoint
            
        else: 
            self.controller_error.append("setNaN") #otherwise, set error 
        
    def set_output(self,n):
        if self.is_number(n) == True: #check if n is a number
            if n > self.max_output or n < 0.0: #check if in allowed range if not..
                self.output_off()
                self.controller_error.append("OutputOutOfRange") #set error 
            else:
                self.output = self.set + n # have to add setpoint as pilot control / vorsteuerung, 
                dac_output = int(self.output*(4095/20)) #scale 0...20 to 0...4095
                #print(dac_output)
                if dac_output > 4095:
                    dac_output = 4095 #limit to max valid value
                self.DAC.channel_b.raw_value = dac_output #turn DAC to requested value
        else: # if n is not a number
            self.output_off()
            self.controller_error.append("OutputNaN") #set error 


    def output_off(self):
            self.output = 0 #set heating power to 0
            self.DAC.channel_b.raw_value = self.output #turn off heater          

      #  if self.control_active == False and self.T_set > 0: #check if control is not active and set point is equal/lower zero, allow control to reset/startup
      #      self.control_active = True #turn control back on
      #      self.T_controller_error = "Ok" #reset error 
      #      #self.pid.auto_mode = True #activate pid controller 
      #      #self.time_temp_check = time.time() #get current time for time based checks
      #  elif self.T_set <= 0: #if T_set is 
      #      self.control_active = False
      #      self.pid.auto_mode = False #de-activate pid controller 

    #call this function to run control loop, will always return sensor reading, rel. heater power, control_active flag and error list
    #if control is set active it will also run PID loop and set heater accordingly
    def run(self): 
        #self.update_T_set(T_set) #update T_set, not needed for MQTT implementation
        self.read() #read current temperature from sensor
        #print(self.P_read)
        #there might be errors during this, so check if we can continue 
        if len(self.controller_error) != 0: #if there is something in the error list 
            self.output_off() #call function, turns off heater/sets heater power accordingly
            self.control_active = False # set flag that control is not active
        
        #if there are no errors we continue here:
        #check if control_active flag is true 
        elif self.control_active == True:
            self.limit_check() #check for temp limit conditions 
            if len(self.controller_error) != 0: #if there is something in the error list  
                self.output_off() #call function, turns off heater/sets heater power accordingly
                self.control_active = False # set flag that control is not active
              
            else: #if no limit errors, continue:
                #self.output = self.pid(self.P_read) #calculate new PID heater power 
                self.set_output(self.pid(self.voltage_read)) #try to set new power, will catch errors 
                if len(self.controller_error) != 0: #if there is something in the error list  
                    self.output_off() #call function, turns off heater/sets heater power accordingly
                    self.control_active = False # set flag that control is not active

            
        err = self.controller_error.copy() #copy our errors must be a copy otherwise err will point to original list and be cleared too
        self.controller_error.clear() #clear error list for next run 
        return [self.voltage_read, self.current_read, self.output, self.control_active, err]    #return list of sensor reading, rel. heater power and errors

    #call this function to activate/deactivate the control 
    def mode(self, m):
        if isinstance(m, str): #check that is str and turn into bool
            if m == "True":
                self.pid.auto_mode = False
                self.pid.auto_mode = True #toggle pid to start with fresh output/states
                self.time_check = time.time() #update time for next comparison of limits check 
                self.control_active = True
            elif m == "False":
                self.output_off()
                self.pid.auto_mode = False
                self.control_active = False
                



    def limit_check(self):   
        # CALL AFTER read_temperature error check, otherwise self.T_read might NaN error might happen
        # check for overshoot/runaway, does not make sens, will get triggerd by reduction of set point, time vs deviation check should be enough
        #if (self.T_read-self.T_set) >= self.T_dev_up_max:
         #   self.T_controller_error.append("OvershootError") # set error flag
            #print("first check tripped")  # debug only

        
        # check for deviation time out -> delta between T_act and T_set greater T_dev_time_max for time_out_temp minutes    
        if time.time()-self.time_check >= self.time_out*60:
            self.time_check = time.time() #update time for next comparison 
            #print("second check run") # debug only
            #check current deviation 
            if abs(self.voltage_read-self.set) >= self.dev_time_max:
                self.current_dev_time_check = True
            else:
                self.current_dev_time_check = False
            # check if deviation is too big at current time/sample and if it was at the last time/sample, if so set error
            if  self.current_dev_time_check == True and self.last_dev_time_check == True:
                self.controller_error.append("HV_setHV_readDeviationTimeOut") 
                #print("second check tripped")  #debug only
                #print(self.T_controller_error) #debug only
            # update flag for next check 
            self.last_dev_time_check = self.current_dev_time_check




##############################
#### GPIO/IO CONFIGURATION ###
##############################
            
#GPIO config 
cartridge_heater = gpiozero.PWMOutputDevice(18, frequency=5)  # define PWM out for heater power control, see gpiozero documentation
ring_heater = gpiozero.PWMOutputDevice(19, frequency = 5)  # define PWM out for heater power control, see gpiozero documentation

i2c = busio.I2C(board.SCL, board.SDA) #declare I2C bus
mcp4728 = adafruit_mcp4728.MCP4728(i2c) #create MCP4728 instance, DAC on DAC 4 click board

collector_vacuum_GPIO = gpiozero.DigitalOutputDevice(17) #create output pin for step pin
collector_vacuum_GPIO.off() #make sure it is turned off

limit_switch_triggered_GPIO = gpiozero.DigitalInputDevice(27, pull_up = True)
HVPowerSupply_GPIO = gpiozero.DigitalInputDevice(22, pull_up = True)

#MCC128 ADC Board config
mcc_adress = daqhats_utils.select_hat_device(daqhats.HatIDs.MCC_128) #use function to select adress of detected MCC 128 boards
mcc128hat = daqhats.mcc128(mcc_adress) #create hat instance 
input_mode = daqhats.AnalogInputMode.SE # use single ended input mode 
input_range = daqhats.AnalogInputRange.BIP_10V #for now use +/- 10V input range
mcc_option = daqhats.OptionFlags.DEFAULT # use default option flags, returns calibrated voltage reading 
#MCC128 channel assignment
cartridge_TC_mcc_channel = 0 #see MCC128 hardware reference 
ring_TC_mcc_channel = 4
pressure_sensor_mcc_channel = 1 
hvsupply_voltage_mcc_channel = 5
hvsupply_current_mcc_channel = 2 

#########(mcp4728, mcc128hat, pressure_sensor_mcc_channel, mcc_option, pressure_control_para)
### INIT OBJECTS ###
####################

### Parameter lists ###
cartridge_temperature_control_para = {
        ### Min/Max/Limit Variables ###
        "T_abs_max": 130,            # upper temp limit for safety shut off (e.g. detect broken wire)
        "T_abs_min": -1,           # lower temp limit for safety shut off (e.g. detect broken wire)
        "T_dev_up_max": 10,         # max deviation above set point for safety shut off (e.g. overshoot/runaway)
        "T_dev_time_max": 5,      # max deviation from set point for time out 
        "time_out_temp":  3,      # time intervall for check of |(T_act-Tset)| > T_dev_time_max 
        "max_heating_power": 1,    # maximum relative heating power 0...1
        ### PID variables ###
        "pid_P": 0.2,                # PID proportional 1
        "pid_I": 0.005,               # PID integral 0.01
        "pid_D": 0               # PID deviation   
    }

ring_temperature_control_para = {
        ### Min/Max/Limit Variables ###
        "T_abs_max": 130,            # upper temp limit for safety shut off (e.g. detect broken wire)
        "T_abs_min": -1,           # lower temp limit for safety shut off (e.g. detect broken wire)
        "T_dev_up_max": 10,         # max deviation above set point for safety shut off (e.g. overshoot/runaway)
        "T_dev_time_max": 5,      # max deviation from set point for time out 
        "time_out_temp":  5,      # time intervall for check of |(T_act-Tset)| > T_dev_time_max 
        "max_heating_power": 1,    # maximum relative heating power 0...1
        ### PID variables ###
        "pid_P": 0.1,                # PID proportional 1
        "pid_I": 0.005,               # PID integral 0.01
        "pid_D": 0               # PID deviation   
    }

pressure_control_para = {
        ### Min/Max/Limit Variables ###
        "P_abs_max": 10,            # upper temp limit for safety shut off (e.g. detect broken wire)
        "P_abs_min": -1,           # lower temp limit for safety shut off (e.g. detect broken wire)
        "P_dev_up_max": 1,         # max deviation above set point for safety shut off (e.g. overshoot/runaway)
        "P_dev_time_max": 0.5,      # max deviation from set point for time out 
        "time_out":  1,      # time intervall for check of |(T_act-Tset)| > T_dev_time_max 
        "max_output": 10,    # maximum output
        ### PID variables ###
        "pid_P": 0,                # PID proportional 
        "pid_I": 0.2,               # PID integral 
        "pid_D": 0.0               # PID deviation   
    }

hvsupply_control_para = {
        ### Min/Max/Limit Variables ###
        "abs_max": 21,            # upper temp limit for safety shut off (e.g. detect broken wire)
        "abs_min": -1,           # lower temp limit for safety shut off (e.g. detect broken wire)
        "dev_up_max": 1,         # max deviation above set point for safety shut off (e.g. overshoot/runaway)
        "dev_time_max": 2,      # max deviation from set point for time out 
        "time_out":  3,      # time intervall for check of |(T_act-Tset)| > T_dev_time_max 
        "max_output": 20,    # maximum output
        ### PID variables ###
        "pid_P": 2,                # PID proportional 0.1
        "pid_I": 1,               # PID integral , 0.4
        "pid_D": 0               # PID deviation   
    }


### Objects ###
cartridge_temperature_control = temp_controller_mcc_AD8495(cartridge_heater, mcc128hat, cartridge_TC_mcc_channel, mcc_option, cartridge_temperature_control_para)
ring_temperature_control = temp_controller_mcc_AD8495(ring_heater, mcc128hat, ring_TC_mcc_channel, mcc_option, ring_temperature_control_para)
pressure_control = pressure_controller_mcc_MCP4728(mcp4728, mcc128hat, pressure_sensor_mcc_channel, mcc_option, pressure_control_para)
hvsupply_voltage_control = voltage_controller_mcc_MCP4728(mcp4728, mcc128hat, hvsupply_voltage_mcc_channel, hvsupply_current_mcc_channel, mcc_option, hvsupply_control_para)

##################
### INIT MQTT  ###
##################

#MQTT connections config
mqttbroker = "192.168.137.4"
port = 1883
maintopic = "MEWRP4/" #use wildcard to catch all topics
topic1_1 = maintopic + "CartridePrintHead/SET_CartridgeTemperature"
topic1_2 = maintopic + "CartridePrintHead/ACT_CartridgeTemperature" 
topic1_3 = maintopic + "CartridePrintHead/ACT_RelativeHeatingPowerCartridge" 
topic1_4 = maintopic + "CartridePrintHead/CONTROLIN_CartridgeTemperature" 
topic1_5 = maintopic + "CartridePrintHead/CONTROLOUT_CartridgeTemperature" 

topic1_6 = maintopic + "CartridePrintHead/SET_RingTemperature" 
topic1_7 = maintopic + "CartridePrintHead/ACT_RingTemperature" 
topic1_8 = maintopic + "CartridePrintHead/ACT_RelativeHeatingPowerRing" 
topic1_9 = maintopic + "CartridePrintHead/CONTROLIN_RingTemperature" 
topic1_10 = maintopic + "CartridePrintHead/CONTROLOUT_RingTemperature"

topic1_11 = maintopic + "CartridePrintHead/SET_Pressure"
topic1_12 = maintopic + "CartridgePrintHead/ACT_Pressure"
topic1_17 = maintopic + "CartridePrintHead/SET_PressureOFFSET"
topic1_16 = maintopic + "CartridgePrintHead/ACT_PressureWOOFFSET"
topic1_13 = maintopic + "CartridePrintHead/ACT_PressureOutputVoltage" 
topic1_14 = maintopic + "CartridePrintHead/CONTROLIN_Pressure" 
topic1_15 = maintopic + "CartridePrintHead/CONTROLOUT_Pressure" 

#DIO topics
topic2_1 = maintopic + "DIO/CollectorVacuum" 
topic2_2 = maintopic + "DIO/LimitSwitchSafetyTriggered"
topic2_3 = maintopic + "DIO/HVPowerSupply"

#HighVoltage topics
topic3_1 = maintopic + "HighVoltage/SET_HVSupplyVoltage"
topic3_2 = maintopic + "HighVoltage/ACT_HVSupplyVoltage"
topic3_3 = maintopic + "HighVoltage/ACT_HVSupplyCurrent"
#topic3_4 = maintopic + "HighVoltage/ACT_HVVoltageOutputVoltage"
topic3_5 = maintopic + "HighVoltage/CONTROLIN_HVSupply"
topic3_6 = maintopic + "HighVoltage/CONTROLOUT_HVSupply"

#heartbeat variable
heartbeat = True #start with true to allow program start up

#callback function definitions for MQTT 
def nodered_heartbeat_callback(client, userdata, rc):
    global heartbeat 
    heartbeat = True

def cartridge_set_temperature_callback(client, userdata, message):
    payload = message.payload.decode("utf-8")
    try:
        cartridge_temperature_control.update_T_set(float(payload))
    except:
        print("Cartridge set temperature payload invalid, received:" + payload)

def ring_set_temperature_callback(client, userdata, message):
    payload = message.payload.decode("utf-8")
    try:
        ring_temperature_control.update_T_set(float(payload))
    except:
        print("Ring set temperature payload invalid, received:" + payload)

def cartridge_control_input_callback(client, userdata, message):
    payload = message.payload.decode("utf-8")
    if payload == "true":
        cartridge_temperature_control.mode("True") #call temperature controller obj function to activate/deactivate 
    elif payload == "false":
        cartridge_temperature_control.mode("False")
    else:
        print("Cartridge control input payload invalid, received:" + payload)

def ring_control_input_callback(client, userdata, message):
    payload = message.payload.decode("utf-8")
    if payload == "true":
        ring_temperature_control.mode("True") #call temperature controller obj function to activate/deactivate 
    elif payload == "false":
        ring_temperature_control.mode("False")
    else:
        print("Ring control input payload invalid, received:" + payload)

def pressure_set_callback(client, userdata, message):
    payload = message.payload.decode("utf-8")
    try:
        pressure_control.update_P_set(float(payload))
    except:
        print("Pressure set payload invalid, received:" + payload)

def pressure_control_input_callback(client, userdata, message):
    payload = message.payload.decode("utf-8")
    if payload == "true":
        pressure_control.mode("True") #call temperature controller obj function to activate/deactivate 
    elif payload == "false":
        pressure_control.mode("False")
    else:
        print("Pressure control input payload invalid, received:" + payload)

def pressure_offset_callback(client, userdata, message):
    payload = message.payload.decode("utf-8")
    try:
        #print(payload)
        pressure_control.update_offset(float(payload))
    except:
        print("pressure offset payload invalid, received:" + payload)

def collector_vacuum_callback(client, userdata, message):
    payload = message.payload.decode("utf-8")
    if payload == "true":
        collector_vacuum_GPIO.on()
    elif payload == "false":
        collector_vacuum_GPIO.off()
    else:
        print("payload invalid, received:" + payload)

def hvsupply_set_voltage_callback(client, userdata, message):
    payload = message.payload.decode("utf-8")
    try:
        #print(payload)
        hvsupply_voltage_control.update_set(float(payload))
    except:
        print("HV set voltage payload invalid, received:" + payload)

def hvsupply_control_input_callback(client, userdata, message):
    payload = message.payload.decode("utf-8")
    #print(payload)
    if payload == "true":
        hvsupply_voltage_control.mode("True") #call temperature controller obj function to activate/deactivate 
    elif payload == "false":
        hvsupply_voltage_control.mode("False")
    else:
        print("Cartridge control input payload invalid, received:" + payload)    

### INIT MQTT COMMUNICATION ###
client = mqtt.Client(protocol=mqtt.MQTTv5)  # create mqtt client, make sure to use v5 protocol 
client.connect(mqttbroker, port) #connect to broker using the defined adress and port
client.loop_start() #start the background loop to process messages 
client.subscribe([(maintopic + "#", 2), ("Diagnostics/#", 2)]) #subscribe to the topic with QOS2 
client.message_callback_add("Diagnostics/NodeRedHeartbeat", nodered_heartbeat_callback)
client.message_callback_add(topic1_1, cartridge_set_temperature_callback) # add callback for specific sub-topic
client.message_callback_add(topic1_4, cartridge_control_input_callback) # add callback for specific sub-topic
client.message_callback_add(topic1_6, ring_set_temperature_callback) # add callback for specific sub-topic
client.message_callback_add(topic1_9, ring_control_input_callback) # add callback for specific sub-topic
client.message_callback_add(topic1_11, pressure_set_callback)
client.message_callback_add(topic1_14, pressure_control_input_callback)
client.message_callback_add(topic1_17, pressure_offset_callback)

client.message_callback_add(topic2_1, collector_vacuum_callback)

client.message_callback_add(topic3_1, hvsupply_set_voltage_callback)
client.message_callback_add(topic3_5, hvsupply_control_input_callback)

### INIT MCC BOARD ###
mcc128hat.a_in_mode_write(input_mode) #set input mode
mcc128hat.a_in_range_write(input_range) #set input range 

### INIT MCP4728 DAC
mcp4728.channel_a.vref = adafruit_mcp4728.Vref.INTERNAL #use internatl reference voltage
mcp4728.channel_a.gain = 1 #set gain
mcp4728.channel_b.vref = adafruit_mcp4728.Vref.INTERNAL #use internatl reference voltage
mcp4728.channel_b.gain = 1 #set gain

#################
### Main loop ###
#################

heartbeat_time = time.monotonic()
control_loop_period = 0.02 #defines how often the control loop should run 0.02 s -> 20 ms or 50 Hz 
while True:
    try:
        starttime = time.monotonic()
        currenttime = datetime.datetime.utcnow().timestamp() #generate UTC Unix timestamp
        #Cartridge Temperature control 
        hc_ret = cartridge_temperature_control.run() #run the heater control loop, give it the set point and take return
       # print(cartridge_temperature_control.T_read)
        if len(hc_ret) == 4: #should be in controller class, maybe later
            # write messages to mqtt, hc_ret has structure of:
            #return [self.T_read, self.heater_power, str(self.control_active), err] 
            
            client.publish(topic1_2, (str(currenttime) + "," + str(hc_ret.pop(0))), qos=2) #write sensor reading to mqtt with QOS 2  
            client.publish(topic1_3, (str(currenttime) + "," + str(hc_ret.pop(0))), qos=2) #write rel. heating power to pipe
            status_list = "Temperature control active:" + str(hc_ret.pop(0)) + ","#write status to output pipe
            err_list = ""
            for err in hc_ret.pop(0):
                err_list += str(err) + ","
            if len(err_list) != 0: # if there are errors..
                status_list += " Temperature controll errors:," + err_list
            client.publish(topic1_5, (str(currenttime) + "," + status_list), qos=2)
        
        #Ring Temperature control
        hc_ret = ring_temperature_control.run() #run the heater control loop, give it the set point and take return
        
        if len(hc_ret) == 4: #should be in controller class, maybe later
            # write messages to mqtt, hc_ret has structure of:
            #return [self.T_read, self.heater_power, str(self.control_active), err] 
            
            client.publish(topic1_7, (str(currenttime) + "," + str(hc_ret.pop(0))), qos=2) #write sensor reading to mqtt with QOS 2  
            client.publish(topic1_8, (str(currenttime) + "," + str(hc_ret.pop(0))), qos=2) #write rel. heating power to pipe
            status_list = "Temperature control active:" + str(hc_ret.pop(0)) + "," #write status to output pipe
            err_list = ""
            for err in hc_ret.pop(0):
                err_list += str(err) + ","
            if len(err_list) != 0: # if there are errors..
                status_list += " Temperature controll errors:," + err_list
            client.publish(topic1_10, (str(currenttime) + "," + status_list), qos=2)

        #Pressure control
        hc_ret = pressure_control.run() #run the heater control loop, give it the set point and take return
        if len(hc_ret) == 4: #should be in controller class, maybe later
            # write messages to mqtt, hc_ret has structure of:
            #return [self.T_read, self.heater_power, str(self.control_active), err] 
            
            client.publish(topic1_12, (str(currenttime) + "," + str(hc_ret.pop(0))), qos=2) #write sensor reading to mqtt with QOS 2  
            client.publish(topic1_13, (str(currenttime) + "," + str(hc_ret.pop(0))), qos=2) #write rel. heating power to pipe
            status_list = "Pressure control active:" + str(hc_ret.pop(0)) + "," #write status to output pipe
            err_list = ""
            for err in hc_ret.pop(0):
                err_list += str(err) + ","
            if len(err_list) != 0: # if there are errors..
                status_list += "Pressure controll errors:," + err_list
            client.publish(topic1_15, (str(currenttime) + "," + status_list), qos=2)  
            client.publish(topic1_16, (str(currenttime) + "," + str(pressure_control.P_read_wooffset)), qos=2) 

        #High voltage control
        hc_ret = hvsupply_voltage_control.run() #run the heater control loop, give it the set point and take return
        if len(hc_ret) == 5: #should be in controller class, maybe later
            # write messages to mqtt, hc_ret has structure of:
            #return [self.T_read, self.heater_power, str(self.control_active), err] 
            
            client.publish(topic3_2, (str(currenttime) + "," + str(hc_ret.pop(0))), qos=2) #write sensor reading to mqtt with QOS 2
            client.publish(topic3_3, (str(currenttime) + "," + str(hc_ret.pop(0))), qos=2) #write sensor reading to mqtt with QOS 2 
            _ =hc_ret.pop(0) 
            #client.publish(topic3_4, (str(currenttime) + "," + str(hc_ret.pop(0))), qos=2) #write rel. heating power to pipe
            status_list = "HV control active:" + str(hc_ret.pop(0)) + "," #write status to output pipe
            err_list = ""
            for err in hc_ret.pop(0):
                err_list += str(err) + ","
            if len(err_list) != 0: # if there are errors..
                status_list += "HV controll errors:," + err_list
            client.publish(topic3_6, (str(currenttime) + "," + status_list), qos=2)  

        #Limit switch GPIO reading
        client.publish(topic2_2, limit_switch_triggered_GPIO.value, qos=2) #read GPIO and write to mqtt
        client.publish(topic2_3, HVPowerSupply_GPIO.value, qos=2) #read GPIO and write to mqtt

        #NodeRed Heartbeat check
        if (starttime-heartbeat_time) >= 30: # if at least x seconds passed, check if heartbeat is True 
            if heartbeat == False: # if not true, raise exception and stopp program
                raise Exception()
            else: 
                heartbeat = False #if heartbeat was there, turn it to false so that next heartbeat msg can reset it 
                heartbeat_time = time.monotonic() #save new heartbeat time
        
        #time.sleep(0.2) #debug only
        processing_time = (time.monotonic() - starttime) #calculate processing time
        client.publish("Diagnostics/MEWRP4ProcessingTime", processing_time, qos=2)
        if processing_time < control_loop_period: 
            #print("sleep")
            time.sleep(control_loop_period - ((time.monotonic() - starttime) % control_loop_period)) #for now 

    except: #catch execptions e.g.keyboard interrupt to make sure we turn off heater before we close program 
        cartridge_heater.value = 0 #turn off heater 
        ring_heater.value = 0 #turn off heater 
        mcp4728.channel_a.raw_value = 0 # turn off pressure
        mcp4728.channel_b.raw_value = 0 # turn off HV
        client.unsubscribe([maintopic, "Diagnostics/NodeRedHeartbeat"])
        client.disconnect()
        client.loop_stop()
        print("stopping")
        if heartbeat == False:
            print("NodeRed heartbeat missing")
        try:
            sys.exit(1)
        except:
            os._exit(1)