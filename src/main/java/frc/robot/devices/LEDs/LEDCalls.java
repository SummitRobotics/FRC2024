/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.devices.LEDs;

import frc.robot.utilities.lists.Colors;

/**
 * all the priorities for the LEDs.
 */
public class LEDCalls {
    enum Priority {
        ON(0),
        MO(2),
        COOP(3),
        AMPLIFY(4),
        IDLE(5),
        RECEIVING(6);

        public int value;

        Priority(int value) {
            this.value = value;
        }
    }

    public static final LEDCall ON = new LEDCall(Priority.ON.value, LEDRange.All).solid(Colors.CHAOS_THEORY_GREEN),
            NOTE_DETECTED = new LEDCall(Priority.NOTE_DETECTED.value, LEDRange.All).flashing(Colors.PURPLE, Colors.OFF),
            MO = new LEDCall(Priority.MO.value, LEDRange.All).sine(Colors.PINK),
            COOP = new LEDCall(Priority.COOP.value, LEDRange.All).flashing(Colors.YELLOW, Colors.OFF),
            AMPLIFY_BLUE = new LEDCall(Priority.AMPLIFY.value, LEDRange.All).flashing(Colors.BLUE, Colors.OFF),
            AMPLIFY_RED = new LEDCall(Priority.AMPLIFY.value, LEDRange.All).flashing(Colors.RED, Colors.OFF),
            IDLE = new LEDCall(Priority.IDLE.value, LEDRange.All).solid(Colors.CHAOS_THEORY_GREEN),
            RECEIVING = new LEDCall(Priority.RECEIVING.value, LEDRange.All).flashing(Colors.PURPLE, Colors.OFF);
    // LOW_GEAR = new LEDCall(Priority.LOW_GEAR.value,
    // LEDRange.All).sine(Colors.RED),
    // INTAKE_DOWN = new LEDCall(Priority.INTAKE_DOWN.value,
    // LEDRange.Middle).flashing(Colors.BLUE, Colors.OFF),
    // ARM_LOW = new LEDCall(Priority.ARM_PRESETS.value,
    // LEDRange.All).ffh(Colors.ORANGE, Colors.OFF),
    // ARM_MID = new LEDCall(Priority.ARM_PRESETS.value,
    // LEDRange.All).ffh(Colors.RED, Colors.OFF),
    // ARM_HIGH = new LEDCall(Priority.ARM_PRESETS.value,
    // LEDRange.All).ffh(Colors.PINK, Colors.OFF),
    // CARDIANL_SELECT = new LEDCall(Priority.ARM_CARDINAL_SELECT.value,
    // LEDRange.All).fff(Colors.GREEN, Colors.OFF),

    // HOMMED_STATUS = new LEDCall(Priority.HOMMED_STATUS.value,
    // LEDRange.All).flashing(Colors.ORANGE, Colors.PINK),
    // CONE_HP = new LEDCall(Priority.HP_SIGNAL.value,
    // LEDRange.HumanPlayer).flashing(Colors.YELLOW, Colors.OFF),
    // CUBE_HP = new LEDCall(Priority.HP_SIGNAL.value,
    // LEDRange.HumanPlayer).flashing(Colors.PURPLE, Colors.OFF);
}
