package frc.robot;

public class States 
{

    public static enum DriveStates 
    {
        standard, leftHold, rightHold, forwardHold, backwardHold, DynamicLock
    }

    public static enum AlignedStates 
    {
        aligned, unAligned, normal
    }

    public static enum ElevatorStates 
    {
        hold, coral0, coral1, coral2
    }

    public static enum ClimberStates 
    {
        hold, up, down
    }

    public static DriveStates driveState = DriveStates.standard;
    public static AlignedStates alignedState = AlignedStates.normal;
    public static ElevatorStates elevatorState = ElevatorStates.hold;
    public static ElevatorStates climberState = ElevatorStates.hold;
}
