package frc.robot.subsystems.Elevator;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;


public class RevThroughBoreEncoder 
{
    private DutyCycleEncoder m_dutyCycleEncoder;
    private Rotation2d m_offset = Rotation2d.fromDegrees(0);
    private boolean m_Inverted;
    
    public RevThroughBoreEncoder(int dioChannel) 
    {
        m_dutyCycleEncoder = new DutyCycleEncoder(dioChannel);
        m_dutyCycleEncoder.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0);
    }

    public Rotation2d getOffset() 
    {
        return m_offset;
    }

    public void setOffset(Rotation2d offset) 
    {
        this.m_offset = offset;
    }

    public boolean isInverted() 
    {
        return m_Inverted;
    }

    public void setInverted(boolean inverted) 
    {
        m_Inverted = inverted;
    }

    public boolean isConnected() 
    {
        return m_dutyCycleEncoder.isConnected();
    }

    public Rotation2d getAngle() 
    {
        double angle = (m_dutyCycleEncoder.get() * 360.0) % 360; // Updated API call to get()
        
        angle -= m_offset.getDegrees();
        
        if (m_Inverted) 
        {
            angle = 360 - angle;
        }
        
        if (angle < 0.0) 
        {
            angle += 360;
        }
        
        return Rotation2d.fromDegrees(angle % 360);
    }
}