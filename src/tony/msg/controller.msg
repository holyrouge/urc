############################################
#Digital buttons constants
#############################################
int32 A=0
int32 B=1
int32 X=2
int32 Y=3
int32 RB=5
int32 LB=4
int32 SELECT=6
int32 START=7
int32 XBOX=8
int32 L3=9 # left stick press down button
int32 R3=10 # right stick press down button

#Analog button constants
int32 LS_X=0
int32 LS_Y=1
int32 RS_X=2
int32 RS_Y=3
int32 RT=4
int32 LT=5
int32 DPAD_X=6
int32 DPAD_Y=7

int32 NUM_BUTTONS=11

############################################
#Variables
############################################
bool[11] isPressed
int16 stickL_x
int16 stickL_y
int16 stickR_x
int16 stickR_y
int16 dpad_x
int16 dpad_y
int16 lt
int16 rt
int16 type