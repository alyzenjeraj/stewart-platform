from inputs import get_gamepad
goal_orientation = [0,0,0]
while 1:
    #print("Press any key to continue! (or press ESC to quit!)")
    #if getch() == chr(0x1b):
    #    break
    events = get_gamepad()
    for event in events:
        if(event.ev_type == "Absolute"):
            if(event.code == "ABS_X"):
                goal_orientation[1] = (int(event.state) - 128) / 10
            if(event.code == "ABS_Y"):
                goal_orientation[2] = (int(event.state) - 128) / 10
            if(event.code == "ABS_GAS"):
                goal_orientation[0] = int(event.state) / 10
    target_angle = 15
    print(goal_orientation)

# "Absolute" "ABS_X/Y" 0~255
# "Absolute" "ABS_GAS" 0~255