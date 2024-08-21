MODULE Module_Left_Arm_IRB14000
    !***********************************************************
    !
    ! Module     :  Module_Left_Arm_IRB14000
    ! Description:
    !   ABB YuMi IRB14000 Left Arm Module for Externally Guided Motion [EGM]
    !   using UdpUc external sensor - setup for communication over
    !   Robot Operating System (ROS)
    ! Author     : Alushi
    ! Supervisor : K.Al Khudir
    ! Version    : 1.0
    !
    !***********************************************************
    
    !***********************************************************
    ! Procedure main
    !   This is the program entry point 
    !***********************************************************
    
VAR egmident channel_1;
    VAR egmstate egmSt1;
    ! Adjust these constants to the limits that make sense for your application.
    CONST egm_minmax egm_minmax_lin1:=[-1,1]; ! in mm
    CONST egm_minmax egm_minmax_rot1:=[-2,2]; ! in degrees
    CONST egm_minmax egm_minmax_joint1:=[-0.5,0.5]; ! in radians or degrees based on your system setup

  PROC main()
        VAR jointtarget joints;
        ! "Move" a tiny amount to start EGM; externally guided motion may only begin from fine point.
        joints:= CJointT();                   ! Reads current joint angles for move to current pose
        MoveAbsj joints, v1000, fine, tool0;  ! Ensure velocity and zone data are appropriate for your robot.
        StartEGM;       
        ! Start EGM action here:
        EGMActJoint channel_1 \Tool:=tool0 \WObj:=wobj0, \J1:=egm_minmax_joint1 \J2:=egm_minmax_joint1 \J3:=egm_minmax_joint1
        \J4:=egm_minmax_joint1 \J5:=egm_minmax_joint1 \J6:=egm_minmax_joint1 \J7:=egm_minmax_joint1 \LpFilter:=100 \Samplerate:=4 \MaxPosDeviation:=1000 \MaxSpeedDeviation:=10000;
        ! Start the EGM joint movement.
        EGMRunJoint channel_1, EGM_STOP_HOLD \J1 \J2 \J3 \J4 \J5 \J6 \J7 \CondTime:=2000000 \RampInTime:=0.01 \PosCorrGain:=0;
        ! An infinite loop to keep the robot running in EGM
        WHILE TRUE DO
            WaitTime 1;                       ! Adjustable for suitable cycle time for application.
        ENDWHILE
        ExitCycle;                            ! Exit the procedure on error or interruption.              
    ERROR
        TPWrite "An error occurred.";         ! Handle errors as appropriate for your application.
        RAISE;
    ENDPROC
    
    PROC StartEGM()
        EGMGetId channel_1;                   ! Obtain a unique EGM identifier.
        egmSt1 := EGMGetState(channel_1);
        IF egmSt1 <= EGM_STATE_CONNECTED THEN ! Set up EGM if the robot is not yet connected.
            EGMSetupUC ROB_L, channel_1, "ConfL", "UCdeviceL:" \Joint \CommTimeout:=10000;
        ENDIF
        EGMStreamStart channel_1;             ! Start EGM stream.
    ENDPROC
    
ENDMODULE