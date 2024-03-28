package frc.lib.leds;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AddressableLEDs extends SubsystemBase {
    
    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;

    private Map<Class<? extends LEDPattern>, LEDPattern> ledPatterns;
    private LEDPattern ledPattern;

    public AddressableLEDs (int port, int leds) {

        this.ledStrip = new AddressableLED(port);
        this.ledBuffer = new AddressableLEDBuffer(leds);
        this.ledPatterns = new HashMap<>();

        this.ledStrip.setLength(leds);
        this.ledStrip.start();
    }

    @Override
    public void periodic () {

        this.ledPattern.runPattern(this.ledBuffer);
        this.ledStrip.setData(this.ledBuffer);
    }

    public void setPattern (Class<? extends LEDPattern> newPattern) {

        this.ledPattern = this.ledPatterns.get(newPattern);

        if (ledPattern == null) {

            try {

                this.ledPattern = (LEDPattern) newPattern.getConstructors()[0].newInstance();
                this.ledPatterns.put(newPattern, this.ledPattern);
            } catch (Exception exception) {

                exception.printStackTrace();
            }
        }
    }
}
