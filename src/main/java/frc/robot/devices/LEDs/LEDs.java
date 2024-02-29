package frc.robot.devices.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.LEDs.LEDRange.Atomic;
// import frc.robot.utilities.lists.Ports;
import java.util.HashMap;
import java.util.UUID;

/**
 * Subsystem for the LEDs on the robot.
 */
public class LEDs extends SubsystemBase {

    private static LEDs instance = null;

    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer buffer;

    private final HashMap<String, LEDCall> calls;
    private boolean callsOutOfDate;

    private int loop;

    /**
     * Gets the LED instance using the singleton pattern.
     *
     * @return the LED instance
     */
    public static LEDs getInstance() {
        if (instance == null) {
            instance = new LEDs();
        }

        return instance;
    }

    /**
     * Creates a new LED management class, based on the robot configuration.
     */
    private LEDs() {
        int LED_PORT = 0;
        int LED_LENGTH = 12;

        ledStrip = new AddressableLED(LED_PORT);
        buffer = new AddressableLEDBuffer(LED_LENGTH);

        ledStrip.setLength(LED_LENGTH);
        ledStrip.start();

        calls = new HashMap<>();
        callsOutOfDate = true;

        loop = 0;
    }

    /**
     * Applies modifications made to the LED buffer to the LED strip.
     */
    private void applyChanges() {
        ledStrip.setData(buffer);
    }

    /**
     * Adds a call to the currently active calls.
     *
     * @param name the call's name (for removal later)
     * @param call the LEDCall object
     */
    public void addCall(String name, LEDCall call) {
        callsOutOfDate = true;
        calls.putIfAbsent(name, call);
    }

    /**
     * Removes a call.
     *
     * @param name the call's name
     */
    public void removeCall(String name) {
        callsOutOfDate = true;
        calls.remove(name);
    }

    /**
     * Removes all calls.
     */
    public void removeAllCalls() {
        callsOutOfDate = true;
        calls.clear();
    }

    /**
     * Reassigns the states of the LED atoms, to assure that priorities up-to date.
     * Runs whenever the currently active calls are modified
     */
    private void reassignCalls() {

        // Refreshes the LED atoms to a blank state
        for (Atomic atom : LEDRange.Atomic.values()) {
            atom.refreshCalls();
        }

        // Updates calls
        for (LEDCall call : calls.values()) {
            for (LEDRange.Atomic atom : call.getRange().getAtoms()) {
                atom.updateCall(call);
            }
        }
    }

    /**
     * Generates a unique string ID for a unnamed LEDCall.
     *
     * @return the unique ID
     */
    public String getUniqueID() {
        while (true) {
            String potentialID = UUID.randomUUID().toString();
            if (!calls.containsKey(potentialID)) {
                return potentialID;
            }
        }
    }

    @Override
    public void periodic() {
        System.out.println("LEDs periodic");
        loop++;

        // Reassigns call if they have been modified
        if (callsOutOfDate) {
            reassignCalls();
            callsOutOfDate = false;
        }

        // Gets LED states
        for (LEDRange.Atomic atom : LEDRange.Atomic.values()) {
            atom.updateLEDs(buffer, loop);
        }

        applyChanges();
    }
}