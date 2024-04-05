package frc.lib.leds.patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.lib.leds.LEDPattern;

public class Rainbow implements LEDPattern {
    
    private int i = 0;
    private int hueOffset = 0;

    public void runPattern (AddressableLEDBuffer buffer) {

        this.i = 0;
        this.handleForwardSegment(buffer, 20);
        this.handleBackwardSegment(buffer, 20);
        this.handleForwardSegment(buffer, 18);
        this.handleBackwardSegment(buffer, 18);

        this.hueOffset += 2;
        this.hueOffset %= 180;
    }

    private void handleForwardSegment (AddressableLEDBuffer buffer, int segmentSize) {

        for (int j = 0; j < segmentSize; j++) {

            int hue = (this.hueOffset + (j * 180 / segmentSize)) % 180;
            buffer.setHSV(this.i, hue, 255, 128);
            this.i++;
        }
    }

    private void handleBackwardSegment (AddressableLEDBuffer buffer, int segmentSize) {

        for (int j = segmentSize - 1; j >= 0; j--) {

            int hue = (this.hueOffset + (j * 180 / segmentSize)) % 180;
            buffer.setHSV(this.i, hue, 255, 128);
            this.i++;
        }
    }
}
