package frc.lib;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

public class ThroughBoreEncoder {
    
    private final DutyCycleEncoder encoder;
    private final Timer startupTimer;
    private boolean initialized;

    private double lastEncoderReading;
    private double position;

    public ThroughBoreEncoder (int channel) {

        this.encoder = new DutyCycleEncoder(channel);
        this.startupTimer = new Timer();
        this.initialized = false;

        this.startupTimer.start();

        this.lastEncoderReading = 0.0;
        this.position = 0.0;
    }

    public void poll () {

        if (this.startupTimer.get() < 1.0) { return; }

        if (!this.initialized) {

            this.lastEncoderReading = this.encoder.getAbsolutePosition();
            this.position = this.lastEncoderReading;
            this.initialized = true;
        }

        double encoderReading = this.encoder.getAbsolutePosition();
        double encoderShift = encoderReading - this.lastEncoderReading;

        if (encoderShift < -0.5) {

            double gain = (1.0 - this.lastEncoderReading) + encoderReading;
            this.position += gain;
        } else if (encoderShift > 0.5) {

            double loss = encoderReading + (1.0 - this.lastEncoderReading);
            this.position -= loss;
        } else {

            this.position += encoderShift;
        }

        this.lastEncoderReading = encoderReading;
    }

    public double get () { return this.position; }
}
