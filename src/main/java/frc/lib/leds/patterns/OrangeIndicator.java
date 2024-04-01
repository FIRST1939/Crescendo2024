package frc.lib.leds.patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.lib.leds.LEDPattern;

public class OrangeIndicator implements LEDPattern {
    
    private int i = 0;
    public static double INDICATOR = 0.0;

    public void runPattern (AddressableLEDBuffer buffer) {
        
        this.i = 0;
        this.handleBackwardSegment(buffer, 20);
        this.handleForwardSegment(buffer, 20);
        this.handleBackwardSegment(buffer, 18);
        this.handleForwardSegment(buffer, 18);
    }

    private void handleForwardSegment (AddressableLEDBuffer buffer, int segmentSize) {
        
        for (int j = 1; j <= segmentSize; i++) {
            
            if ((j * 1.0) / segmentSize <= INDICATOR) {

                buffer.setRGB(this.i, 255, 25, 0);
            } else {

                buffer.setRGB(this.i, 0, 0, 0);
            }

            this.i++;
        }
    }

    private void handleBackwardSegment (AddressableLEDBuffer buffer, int segmentSize) {
        
        for (int j = segmentSize; j >= 1; i--) {
            
            if ((j * 1.0) / segmentSize <= INDICATOR) {

                buffer.setRGB(this.i, 255, 25, 0);
            } else {

                buffer.setRGB(this.i, 0, 0, 0);
            }

            this.i++;
        }
    }
}
