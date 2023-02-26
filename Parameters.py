#!/usr/bin/env python3

import pymodbus
import datetime
import time
from pymodbus.pdu import ModbusRequest
from pymodbus.client import ModbusSerialClient
from pymodbus.transaction import ModbusRtuFramer
from pymodbus.payload import BinaryPayloadBuilder, Endian, BinaryPayloadDecoder


##########----------------------Define Modbus Slave ID of motor ---------------------------------------------------##########

#Powertrain motors
Front_motor_LH_salveid=21
Front_motor_RH_salveid=22
Rear_motor_LH_salveid=23
Rear_motor_RH_salveid=24

#Robot Attachments
Hopper_Z_axis_slaveid=1
Hopper_gripper_slaveid=2
X_axis_slaveid=3
Y_axis_slaveid=4
Z_axis_slaveid=5
Sapling_gripper_slaveid=6

##########---------------------------------------------------------------------------------------------------------##########

##########----------------------Set operating mode of motor -------------------------------------------------------##########

Position_mode=0x001
Velocity_Mode=0x003
Homing_Mode=0x006

##########---------------------------------------------------------------------------------------------------------##########

#########--------------------------Motor Speed  / Acceleration / Deaaceleration------------------------------------##########

Robot_fwd_rpm = 100   #set this value in rpm
Robot_rev_rpm = 100   #set this value in rpm ( set it same as robot fwd rpm for differential action)
Robot_fwd_speed = int(((Robot_fwd_rpm)/60)*10 )
Robot_rev_speed = int(((Robot_rev_rpm)/60)*10 )
Robot_Motion_acceleration = 500  
Robot_Motion_deacceleration = 500

Hopper_Z_axis_rpm = 30
Hopper_Z_Axis_speed = int(((Hopper_Z_axis_rpm)/60)*10 ) # Set is 10 times the rps , example 3000 rpm set is as (3000/60)=50 rps  , so we need to set is as 50*10 = 500
Hopper_Z_Axis_acceleration = 1000  #Set this value to 100 rps/s, which the drive internally sees as 10 rps/s
Hopper_Z_Axis_deacceleration = 1000  #Set this value to 100 rps/s, which the drive internally sees as 10 rps/s

Hopper_Planter_speed = 10  #This value is 10 times the fraction of required value
Hopper_Planter_acceleration = 10
Hopper_Planter_deacceleration = 10

X_Axis_speed = 10  #This value is 10 times the actual value
X_Axis_acceleration = 10  #This value is 10 times the actual value
X_Axis_deacceleration = 10 #This value is 10 times the actual value

Y_Axis_speed = 10  #This value is 10 times the actual value
Y_Axis_acceleration = 10  #This value is 10 times the actual value
Y_Axis_deacceleration = 10 #This value is 10 times the actual value

Z_Axis_speed = 10  #This value is 10 times the actual value
Z_Axis_acceleration = 10  #This value is 10 times the actual value
Z_Axis_deacceleration = 10 #This value is 10 times the actual value

Sapling_gripper_speed = 10  #This value is 10 times the actual value
Sapling_gripper_acceleration = 10  #This value is 10 times the actual value
Sapling_gripper_deacceleration = 10 #This value is 10 times the actual value

##########---------------------------------------------------------------------------------------------------------##########

##########----------------------Establish Connection with Modbus Client -------------------------------------------##########

client = ModbusSerialClient(method='rtu', port="COM9", baudrate=57600, parity='E', timeout=0.1)
connection = client.connect()
print ('\nThe Connection established with system is :' , client.connect())

##########---------------------------------------------------------------------------------------------------------##########

##########-------------------Release motor Brake-------------------------------------------------------------------##########

def Release_motor_brake(SlaveID):
    write  = client.write_register(0x6040,0x0003,SlaveID)  #Release the brake 

##########---------------------------------------------------------------------------------------------------------##########

##########-------------------Initalize the motor after power on ---------------------------------------------------##########

def Intialize_motor(SlaveID):
    client.write_register(0x6040,0x0001,SlaveID)  #Drive Initialization 
    client.write_register(0x6040,0x0003,SlaveID)  #Release the brake 
    client.write_register(0x6040,0x000F,SlaveID)  #Power the Motor Operation Enabled

##########---------------------------------------------------------------------------------------------------------##########


#########------------------------Set Operating mode of motor to Position / Velocity /Homing Mode--------------------##########


def Set_Motor_operating_mode(SlaveID,Operating_mode):   #initialize motor and set its operating mode
    Intialize_motor(SlaveID)
    client.write_register(0x6060,Operating_mode,SlaveID) #Set position mode


##########---------------------------------------------------------------------------------------------------------##########

#########------------------Function to Read and print Motor Parameters------##########

def Read_Motor_operating_parameters(SlaveID):
    Motor_operating_mode = client.read_holding_registers (0x6061, 1, SlaveID) # Read which mode the drive is currently 
    Mode_decoder = BinaryPayloadDecoder.fromRegisters(Motor_operating_mode.registers,Endian.Big)
    mode=Mode_decoder.decode_16bit_int()
    if mode == 1:
        print("Motor is set to Position mode")
    elif mode == 3:
         print("Motor is set to Velocity mode")
    elif mode == 6:
        print("Motor is set to Homing mode")
    else:
        print('Motor mode is not defined')
    
    
    Velocity = client.read_holding_registers ( 0x6081, 2, SlaveID )  #Read_Set_speed
    Velocity_decoder = BinaryPayloadDecoder.fromRegisters(Velocity.registers,Endian.Big)
    Velocity_value=Velocity_decoder.decode_32bit_int()
    #print( 'Target velocity is  = ' , Velocity_value)
    
    Motor_speed  = client.read_holding_registers ( 0x606C, 2, SlaveID ) #Read_Actual_speed_at_current_time
    speed_decoder = BinaryPayloadDecoder.fromRegisters(Motor_speed.registers,Endian.Big)
    Speed_read=speed_decoder.decode_32bit_int()
    #print( 'Actual speed is' , Speed_read , 'rpm')
    
    Encoder_read= client.read_holding_registers ( 0x6064, 2, SlaveID ) #To read the value from encoder
    Encoder_decoder = BinaryPayloadDecoder.fromRegisters(Encoder_read.registers,Endian.Big)
    Encoder_data=Encoder_decoder.decode_32bit_int()
    #print( 'The current reading of encoder is' , Encoder_data)
    
    Motor_acceleration  = client.read_holding_registers ( 0x6083, 1, SlaveID ) #Read_Set_Acceleration
    acceleration_decoder = BinaryPayloadDecoder.fromRegisters(Motor_acceleration.registers,Endian.Big)
    acceleration_data=acceleration_decoder.decode_16bit_int()
    #print ( 'Motor acceleration is set to' , acceleration_data)

    Motor_Deacceleration  = client.read_holding_registers ( 0x6084, 1, SlaveID )  #Read_Set_Deacceleration
    Deacceleration_decoder = BinaryPayloadDecoder.fromRegisters(Motor_Deacceleration.registers,Endian.Big)
    Deacceleration_data=Deacceleration_decoder.decode_16bit_int()
    #print ( 'Motor acceleration is set to' , Deacceleration_data)
    
    print('Target speed : Actual speed :', (Velocity_value*6) , 'rpm',':', Speed_read ,'rpm')
    #print('Actual speed is' , Speed_read ,'rpm')
    #print('Acceleration is : ', acceleration_data )
    #print('Deacceleration is :', Deacceleration_data)
    #print('Acceleration : ', acceleration_data ,'Deacceleration :', Deacceleration_data)
    print('Encoder_reading_is : ', Encoder_data )
    
    
##########----------------------------------------------------------------------------------------------------------##########

#########------------------Function to Read Encoder_data------##########

##########---------------------------------------------------------------------------------------------------------##########
def Read_Encoder_Data(SlaveID):
    Encoder_read= client.read_holding_registers ( 0x6064, 2, SlaveID ) #To read the value from encoder
    Encoder_decoder = BinaryPayloadDecoder.fromRegisters(Encoder_read.registers,Endian.Big)
    Encoder_data=Encoder_decoder.decode_32bit_int()
    #print( 'The current reading of encoder is' , Encoder_data)
    return Encoder_data

#########------------------Function to set :- Position / Speed/Acceleration/Deacceleration for motor---------------##########

def Set_Target_Position(SlaveID,Position_value ):
    builder = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
    builder.reset()
    builder.add_32bit_int(Position_value)
    payload = builder.to_registers()
    client.write_registers(0x607A, payload,SlaveID ) 


def Set_Target_speed(SlaveID,target_velocity):
    builder = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
    builder.reset()
    builder.add_32bit_int(target_velocity)
    payload = builder.to_registers()
    client.write_registers(0x6081, payload,SlaveID ) 
       
def Set_Motor_Acceleration(SlaveID,Motor_acceleration):
    write  = client.write_register(0x6083,Motor_acceleration,SlaveID) # set acceleration
    
def Set_Motor_Deacceleration(SlaveID,Motor_deacceleration):
    write  = client.write_register(0x6084,Motor_deacceleration,SlaveID) # set deceleration


def Set_Motor_Quick_stop_deaaceleration(SlaveID):
    write  = client.write_register(0x6085,400,SlaveID) # set quick stop deacceleration

    
##########----------------------------------------------------------------------------------------------------------##########


#########--------------------------Function to Start/stop the motor In Velocity Mode ---------------------------------##########


def Start_Motor_Run_Position_mode(SlaveID):
    write  = client.write_register(0x6040,0x001F,SlaveID) # Sampling new position in absolute positioning
    write  = client.write_register(0x6040,0x000F,SlaveID) # Get ready for the next fast sampling in absolute positioning
    
def Start_Motor_Run_Relative_mode(SlaveID):  #Relative motion mode
    write  = client.write_register(0x6040,0x025F,SlaveID) # Sampling new position in relative positioning
    write  = client.write_register(0x6040,0x024F,SlaveID) # Get ready for the next fast sampling in relative positioning

def Start_Motor_Run_Speed_mode(SlaveID):
    write  = client.write_register(0x6040,0x000F,SlaveID) # Start running motor
    
def Stop_Motor_Run_Speed_mode(SlaveID):
    write  = client.write_register(0x6040,0x010F,SlaveID) # Stop running motor
    
def Quick_Stop_motor(SlaveID):
    write  = client.write_register(0x605A,0x0002,SlaveID) # Quick Stop running motor
    
def Halt_motor(SlaveID):
    write  = client.write_register(0x605D,0x0002,SlaveID) # Halt motor
    
##########--------------------------------------------------------------------------------------------------------------------##########


#########--------------------------Function to Run motor in absolute positioning----------------------------------------------##########

def Run_Motor_in_position_mode(Slaveid,speed,acceleration,deacceleration,position):
    Set_Target_speed(Slaveid,speed)
    Set_Motor_Acceleration(Slaveid,acceleration)
    Set_Motor_Deacceleration(Slaveid,deacceleration)
    Set_Target_Position(Slaveid,position)
    Start_Motor_Run_Position_mode(Slaveid)
##########--------------------------------------------------------------------------------------------------------------------##########


#########--------------------------Function to Run motor in relative positioning----------------------------------------------##########

def Run_Motor_in_relative_mode(Slaveid,speed,acceleration,deacceleration,position):
    Set_Target_speed(Slaveid,speed)
    Set_Motor_Acceleration(Slaveid,acceleration)
    Set_Motor_Deacceleration(Slaveid,deacceleration)
    Set_Target_Position(Slaveid,position)
    Start_Motor_Run_Relative_mode(Slaveid)
##########--------------------------------------------------------------------------------------------------------------------##########


#########--------------------------Function to Set Motor_speed_and Run it ----------------------------------------------------##########

def Run_Motor_in_speed_mode(Slaveid,speed,acceleration,deacceleration):
    Set_Target_speed(Slaveid,speed)
    Set_Motor_Acceleration(Slaveid,acceleration)
    Set_Motor_Deacceleration(Slaveid,deacceleration)
    Start_Motor_Run_Speed_mode(Slaveid)
      
##########--------------------------------------------------------------------------------------------------------------------##########


#########-------------------------- Function to Move Robot Forward in Position Mode-------------------------------------------##########

def Move_Robot_forward(Front_motor_LH_salveid,Rear_motor_LH_salveid,Front_motor_RH_salveid,Rear_motor_RH_salveid,robot_fwd_speed,acceleration,deacceleration,Pitch):
    Run_Motor_in_relative_mode(Front_motor_LH_salveid,robot_fwd_speed,acceleration,deacceleration,Pitch) #Front_motor_LH
    Run_Motor_in_relative_mode(Rear_motor_LH_salveid,robot_fwd_speed,acceleration,deacceleration,Pitch)  #Rear_motor_LH
    Run_Motor_in_relative_mode(Front_motor_RH_salveid,robot_fwd_speed,acceleration,deacceleration,Pitch) #Front_motor_RH
    Run_Motor_in_relative_mode(Rear_motor_RH_salveid,robot_fwd_speed,acceleration,deacceleration,Pitch)  #Rear_motor_RH

#########--------------------------------------------------------------------------------------------------------------------##########


#########-------------------------- Function to Move Robot Reverse in Position Mode -------------------------------------##########

def Move_Robot_Backward(Front_motor_LH_salveid,Rear_motor_LH_salveid,Front_motor_RH_salveid,Rear_motor_RH_salveid,robot_rev_speed,acceleration,deacceleration,Pitch):
    Run_Motor_in_relative_mode(Front_motor_LH_salveid,robot_rev_speed,acceleration,deacceleration,Pitch) #Front_motor_LH
    Run_Motor_in_relative_mode(Rear_motor_LH_salveid,robot_rev_speed,acceleration,deacceleration,Pitch)  #Rear_motor_LH
    Run_Motor_in_relative_mode(Front_motor_RH_salveid,robot_rev_speed,acceleration,deacceleration,Pitch) #Front_motor_RH
    Run_Motor_in_relative_mode(Rear_motor_RH_salveid,robot_rev_speed,acceleration,deacceleration,Pitch)  #Rear_motor_RH

#########----------------------------------------------------------------------------------------------------------------##########


#########-------------------------- Function to Move Robot Left in Position Mode -------------------------------------##########

def Move_Robot_left(Front_motor_LH_salveid,Rear_motor_LH_salveid,Front_motor_RH_salveid,Rear_motor_RH_salveid,robot_fwd_speed,acceleration,deacceleration,Pitch):
    Run_Motor_in_relative_mode(Front_motor_LH_salveid,int(0.3*robot_fwd_speed),acceleration,deacceleration,-abs(Pitch)) #Front_motor_LH
    Run_Motor_in_relative_mode(Rear_motor_LH_salveid,int(0.3*robot_fwd_speed),acceleration,deacceleration,-abs(Pitch))  #Rear_motor_LH
    Run_Motor_in_relative_mode(Front_motor_RH_salveid,int(0.3*robot_fwd_speed),acceleration,deacceleration,Pitch) #Front_motor_RH
    Run_Motor_in_relative_mode(Rear_motor_RH_salveid,int(0.3*robot_fwd_speed),acceleration,deacceleration,Pitch)  #Rear_motor_RH


#########----------------------------------------------------------------------------------------------------------------##########

#########-------------------------- Function to Move Robot Right in Position Mode -------------------------------------##########

def Move_Robot_right(Front_motor_LH_salveid,Rear_motor_LH_salveid,Front_motor_RH_salveid,Rear_motor_RH_salveid,robot_fwd_speed,acceleration,deacceleration,Pitch):
    Run_Motor_in_relative_mode(Front_motor_LH_salveid,int(0.3*robot_fwd_speed),acceleration,deacceleration,Pitch) #Front_motor_LH
    Run_Motor_in_relative_mode(Rear_motor_LH_salveid,int(0.3*robot_fwd_speed),acceleration,deacceleration,Pitch)  #Rear_motor_LH
    Run_Motor_in_relative_mode(Front_motor_RH_salveid,int(0.3*robot_fwd_speed),acceleration,deacceleration,-abs(Pitch)) #Front_motor_RH
    Run_Motor_in_relative_mode(Rear_motor_RH_salveid,int(0.3*robot_fwd_speed),acceleration,deacceleration,-abs(Pitch))  #Rear_motor_RH


#########----------------------------------------------------------------------------------------------------------------##########