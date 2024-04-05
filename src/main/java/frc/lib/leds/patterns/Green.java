package frc.lib.leds.patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.lib.leds.LEDPattern;

public class Green implements LEDPattern {
    
    public void runPattern (AddressableLEDBuffer buffer) {
        
        for (int i = 0; i < buffer.getLength(); i++) {
            
            buffer.setRGB(i, 0, 255, 0);
        }
    }
}
