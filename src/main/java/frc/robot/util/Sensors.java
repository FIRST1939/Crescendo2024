package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;

public class Sensors {
    
    private static final DigitalInput elevatorLowerBound = new DigitalInput(Constants.ElevatorConstants.LOWER_BOUND);
    private static final DigitalInput elevatorUpperBound = new DigitalInput(Constants.ElevatorConstants.UPPER_BOUND);
    private static final DigitalInput indexerStartBeam = new DigitalInput(Constants.IndexerConstants.START_BEAM);
    private static final DigitalInput indexerEndBeam = new DigitalInput(Constants.IndexerConstants.END_BEAM);
    private static final DigitalInput armLowerBound = new DigitalInput(Constants.ArmConstants.LOWER_BOUND);
    private static final DigitalInput armUpperBound = new DigitalInput(Constants.ArmConstants.UPPER_BOUND);

    public static boolean getElevatorLowerBound () {  return elevatorLowerBound.get(); }
    public static boolean getElevatorUpperBound () {  return elevatorUpperBound.get(); }
    public static boolean getIndexerStartBeam () { return indexerStartBeam.get(); }
    public static boolean getIndexerEndBeam () {  return indexerEndBeam.get(); }
    public static boolean getArmLowerBound () { return armLowerBound.get(); }
    public static boolean getArmUpperBound () {  return armUpperBound.get(); }
}
