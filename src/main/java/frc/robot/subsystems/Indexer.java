package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

    private CANSparkMax leaderRoller;
    private CANSparkMax followerRoller;
    private DigitalInput startBeam;
    private DigitalInput endBeam;
}
