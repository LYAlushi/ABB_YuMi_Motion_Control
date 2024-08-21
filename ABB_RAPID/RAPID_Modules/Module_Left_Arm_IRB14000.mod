MODULE Module_Left_Arm_IRB14000
    
!************************************************
! Module      : Module_Left_Arm_IRB14000
! Description : ABB YuMi Left Arm Module for EGM
!               (Externally Guided Motion) using
!               UdpUc external sensor, setup for
!               communication with ROS
!               (Robot Operating System)
! Author      : L.Y.Alushi
! Research    : Motion Control with collision
!               avoidance for kinematically 
!               redundant manipulator
! Supervisor  : Dr K. Al Khudir
! Institution : Coventry University
! Date        : 06/2024

VAR egmident channel_1;                     ! must match name in ROS configuration
VAR egmstate egmSt1;
! Adjustable based on application :
CONST egm_minmax egm_minmax_lin1 := [-1,1];      ! (mm)
CONST egm_minmax egm_minmax_rot1 := [-2,2];      ! (degrees)
CONST egm_minmax egm_minmax_joint1 := [-0.5,0.5];! (radians or degrees, depending on setup)

PROC main()
    VAR jointtarget joints;                      ! declare a variable for current configuration
    joints := CJointT();                         ! assigns current configuration to variable
    MoveAbsJ joints,v1000,fine,tool0;            ! ensure robot in fine position at current configuration
    ! as in fine position, EGM session may start:
        StartEGM;
    ! Start EGM Action:
    EGMActJoint channel_1 \Tool:=tool0 \WObj:=wobj0, \J1:=egm_minmax_joint1 \J2:=egm_minmax_joint1 \J3:=egm_minmax_joint1 
    \J4:=egm_minmax_joint1 \J5:=egm_minmax_joint1 \J6:=egm_minmax_joint1 \J7:=egm_minmax_joint1 \LpFilter:=100 \Samplerate:=4 \MaxPosDeviation:=1000 \MaxSpeedDeviation:=10000;
    ! An infinite loop to keep the robot running in EGM
        WHILE TRUE DO
            WaitTime 1;                          ! Adjustable for suitable cycle time for application.
        ENDWHILE
        ExitCycle;                               ! Exit the procedure on error or interruption.              
    ERROR
        TPWrite "An error occurred.";            ! Handle errors as appropriate for your application.
        RAISE;
    ENDPROC
    
    PROC StartEGM()
        EGMGetId channel_1;                      ! Obtain a unique EGM identifier.
        egmSt1 := EGMGetState(channel_1);
        IF egmSt1 <= EGM_STATE_CONNECTED THEN    ! Set up EGM if the robot is not yet connected.
            EGMSetupUC ROB_R, channel_1, "ConfL", "UCdeviceL:" \Joint \CommTimeout:=10000;
        ENDIF
        EGMStreamStart channel_1;                ! Start EGM stream.
    ENDPROC
    
ENDMODULE