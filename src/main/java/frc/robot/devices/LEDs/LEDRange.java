package frc.robot.devices.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Options for LED range selection, that are available to the wider robot. Used in the creation of
 * LED ranges.
 */
public enum LEDRange {
    // TODO update led ranges
    Bottom(Atomic.Bottom),
    Middle(Atomic.Middle),
    Top(Atomic.Top),
    Upper(Atomic.Top, Atomic.Middle),
    Lower(Atomic.Middle, Atomic.Bottom),
    All(Atomic.Bottom, Atomic.Middle, Atomic.Top);

    private final Atomic[] ranges;

    LEDRange(Atomic... ranges) {
        this.ranges = ranges;
    }

    public Atomic[] getAtoms() {
        return ranges;
    }

    /**
     * Atomic LED ranges, that make up the smallest segments of LED strip decomposition.
     * They are the building blocks of LEDRanges,
     * and they manage their own internal state to update LED colors.
     */
    protected enum Atomic {
        Bottom(0, 3),
        Middle(4, 7),
        Top(8, 11);

        private final int start;
        private final int end;

        private final Color8Bit defaultColor;
        private LEDCall call;

        /**
         * Creates a new Atom.
         *
         * @param start the starting LED of the strip
         * @param end   the ending LED of the strip
         */
        Atomic(int start, int end) {
            this.start = start;
            this.end = end;

            defaultColor = new Color8Bit(Color.kBlack);
            call = null;
        }

        /**
         * Sets the Atom's active call to null.
         */
        public void refreshCalls() {
            call = null;
        }

        /**
         * Attempts to replace the Atom's call with a new one.
         * If the new call has higher priority, it is accepted.
         *
         * @param newCall the new call
         */
        public void updateCall(LEDCall newCall) {
            if (call == null) {
                call = newCall;
            } else {
                if (call.getPriority() < newCall.getPriority()) {
                    call = newCall;
                }
            }
        }

        /**
         * Updates a buffer using the Atom's call.
         *
         * @param buffer the buffer
         * @param loop   the current loop
         */
        public void updateLEDs(AddressableLEDBuffer buffer, int loop) {
            if (call == null) {
                for (int i = start; i <= end; i++) {
                    buffer.setLED(i, defaultColor);
                }
            } else {
                for (int i = start; i <= end; i++) {
                    buffer.setLED(i, call.getColor(loop, i));
                }
            }
        }
    }
}