package frc.robot.utils;

import edu.wpi.first.wpilibj.DigitalInput;
public class DigitalInputManager {
    private DigitalInput input;
    private boolean currentState;
    private boolean previousState;
    public DigitalInputManager(int channel) {
        input = new DigitalInput(channel);
        currentState = input.get();
        previousState = currentState;
    }
    public void periodic() {
        previousState = currentState;
        currentState = input.get();
    }
    public Boolean pressed() {
        return !previousState && currentState;
    }
}