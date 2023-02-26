
import Parameters
import time


Robot_fwd_pitch = 1500  #Enter distance to travel in mm
Robot_rev_pitch = -1200  #Enter distance to travel in mm


Gear_ratio=32
Wheel_dia=390
Motor_fwd_steps_required=4000*(Gear_ratio*(Robot_fwd_pitch/(2*3.14*(Wheel_dia/2))))
fwd_steps_required= int(round(Motor_fwd_steps_required,3))
#print(Motor_fwd_Steps_required)
Motor_rev_steps_required=4000*(Gear_ratio*(Robot_rev_pitch/(2*3.14*(Wheel_dia/2))))
rev_steps_required= int(round(Motor_rev_steps_required,3))
#print(Motor_rev_Steps_required)



##########-----------------------------------------Initialize motors-----------------------##########

#Front_motor_LH
Parameters.Set_Motor_operating_mode(Parameters.Front_motor_LH_salveid,Parameters.Position_mode)
print('\n**********Front_motor_LH_parameters are**********\n', Parameters.Read_Motor_operating_parameters(Parameters.Front_motor_LH_salveid))

#Front_motor_RH
Parameters.Set_Motor_operating_mode(Parameters.Front_motor_RH_salveid,Parameters.Position_mode)
print('\n**********Front_motor_RH_parameters are**********\n', Parameters.Read_Motor_operating_parameters(Parameters.Front_motor_LH_salveid))

#Rear_motor_LH
Parameters.Set_Motor_operating_mode(Parameters.Rear_motor_LH_salveid,Parameters.Position_mode)
print('\n**********Rear_motor_LH_parameters are**********\n', Parameters.Read_Motor_operating_parameters(Parameters.Front_motor_LH_salveid))

#Rear_motor_RH
Parameters.Set_Motor_operating_mode(Parameters.Rear_motor_RH_salveid,Parameters.Position_mode)
print('\n**********Rear_motor_RH_parameters are**********\n', Parameters.Read_Motor_operating_parameters(Parameters.Front_motor_LH_salveid))

##########----------------------------------------------------------------------------------##########



#########-------------------------- Powertrain : Function to Run Motor Forward -------------------------------------------------##########
    
while True:
    print( '\nStay Away from Robot,it might hit\nFor Forward Motion press : F\nFor Reverse Motion press : B\nFor Left Motion press    : L\nFor Right Motion press   : R' )
   
    choice = input()

    if choice == 'F':
        Parameters.Move_Robot_forward(Parameters.Front_motor_LH_salveid,Parameters.Rear_motor_LH_salveid,Parameters.Front_motor_RH_salveid,Parameters.Rear_motor_RH_salveid,Parameters.Robot_fwd_speed,Parameters.Robot_Motion_acceleration,Parameters.Robot_Motion_deacceleration,fwd_steps_required)
        print("\nForward motion in progress")
        
    elif choice == 'B':
        Parameters.Move_Robot_Backward(Parameters.Front_motor_LH_salveid,Parameters.Rear_motor_LH_salveid,Parameters.Front_motor_RH_salveid,Parameters.Rear_motor_RH_salveid,Parameters.Robot_rev_speed,Parameters.Robot_Motion_acceleration,Parameters.Robot_Motion_deacceleration,rev_steps_required)
        print("\nReverse motion in progress")
        
    elif choice == 'L':
        Parameters.Move_Robot_left(Parameters.Front_motor_LH_salveid,Parameters.Rear_motor_LH_salveid,Parameters.Front_motor_RH_salveid,Parameters.Rear_motor_RH_salveid,Parameters.Robot_fwd_speed,Parameters.Robot_Motion_acceleration,Parameters.Robot_Motion_deacceleration,fwd_steps_required)
        print("\nLeft Motion in progress")
        
    elif choice == 'R':
        Parameters.Move_Robot_right(Parameters.Front_motor_LH_salveid,Parameters.Rear_motor_LH_salveid,Parameters.Front_motor_RH_salveid,Parameters.Rear_motor_RH_salveid,Parameters.Robot_fwd_speed,Parameters.Robot_Motion_acceleration,Parameters.Robot_Motion_deacceleration,fwd_steps_required)
        print("\nRight motion in progress")
        
    else:
        print("\nInvalid choice")
        
    time.sleep(2)   