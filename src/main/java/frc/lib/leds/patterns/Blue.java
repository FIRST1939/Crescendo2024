package frc.lib.leds.patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.lib.leds.LEDPattern;

public class Blue implements LEDPattern {
    
    public void runPattern (AddressableLEDBuffer buffer) {
        
        for (int i = 0; i < buffer.getLength(); i++) {
            
            buffer.setRGB(i, 0, 0, 255);
        }
    }
}
