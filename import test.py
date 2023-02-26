import time
import math

while True:
    print( '\nStay Away from Robot,it might hit\nFor Forward Motion press : F\nFor Reverse Motion press : B\nFor Left Motion press    : L\nFor Right Motion press   : R' )
   # print( 'For Reverse Motion press : B' )
   
    choice = input()

    if choice == 'F':
        print("\nForward motion in progress")
        
    elif choice == 'B':
        print("\nReverse motion in progress")
        
    elif choice == 'L':
        print("\nLeft Motion in progress")
        
    elif choice == 'R':
        print("\nRight motion in progress")
        
    else:
        print("\nInvalid choice")
        
    Gear_ratio=32
    Wheel_dia=390
    wheel_circumference=(2*3.14*(Wheel_dia/2))
    math.floor(wheel_circumference)
    print(wheel_circumference)