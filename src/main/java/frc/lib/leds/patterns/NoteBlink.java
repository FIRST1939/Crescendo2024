package frc.lib.leds.patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.lib.leds.LEDPattern;

public class NoteBlink implements LEDPattern {
    
    private boolean status = true;
    private int loops = 0;

    public void runPattern (AddressableLEDBuffer buffer) {
        
        if (this.loops % 6 == 0) {

            if (status) {

                for (int i = 0; i < buffer.getLength(); i++) {
            
                    buffer.setRGB(i, 0, 0, 0);
                }
            } else {

                for (int i = 0; i < buffer.getLength(); i++) {
            
                    buffer.setRGB(i, 255, 25, 0);
                }
            }
            
            this.status = !this.status;
        }

        this.loops++;
    }
}