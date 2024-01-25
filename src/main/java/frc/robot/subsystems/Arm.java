package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    
    private TalonFX leaderPivot;
    private TalonFX followerPivot;
    private DutyCycleEncoder pivotEncoder;

    private DigitalInput lowerBound;
    private DigitalInput upperBound;
}
