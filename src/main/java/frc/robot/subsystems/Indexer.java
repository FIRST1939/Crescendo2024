package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

public class Indexer extends SubsystemBase {

    private CANSparkMax leaderRollers;
    private CANSparkMax followerRollers;
    private DigitalInput startBeam;
    private DigitalInput endBeam;

    public Indexer () {

        this.leaderRollers = new CANSparkMax(Constants.IndexerConstants.LEADER_ROLLERS, MotorType.kBrushless);
        this.followerRollers = new CANSparkMax(Constants.IndexerConstants.FOLLOWER_ROLLERS, MotorType.kBrushless);

        this.leaderRollers.setInverted(Constants.IndexerConstants.LEADER_ROLLERS_INVERTED);
        this.followerRollers.setInverted(Constants.IndexerConstants.FOLLOWER_ROLLERS_INVERTED);
    }
}
