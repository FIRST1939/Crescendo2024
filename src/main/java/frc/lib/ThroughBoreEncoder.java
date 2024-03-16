package frc.lib;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

public class ThroughBoreEncoder {
    
    private final DutyCycleEncoder encoder;
    private final Timer startupTimer;
    private boolean initialized;

    private double lastEncoderReading;
    private int fullRotations;

    public ThroughBoreEncoder (int channel) {

        this.encoder = new DutyCycleEncoder(channel);
        this.startupTimer = new Timer();
        this.initialized = false;

        this.startupTimer.start();

        this.lastEncoderReading = 0.0;
        this.fullRotations = 0;
    }

    public void poll () {

        if (this.startupTimer.get() < 1.0) { return; }

        if (!this.initialized) {

            this.lastEncoderReading = this.encoder.getAbsolutePosition();
            this.initialized = true;
        }

        double encoderReading = this.encoder.getAbsolutePosition();
        double encoderShift = encoderReading - this.lastEncoderReading;

        if (encoderShift < -0.5) { this.fullRotations++; } 
        else if (encoderShift > 0.5) { this.fullRotations--; }
        this.lastEncoderReading = encoderReading;
    }

    public double get () { return (this.fullRotations + this.lastEncoderReading); }
}
