package com.team1816.lib.inputs;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.GenericHIDSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class ButtonBoardTest {

    private static final int TEST_PORT = 0;

    private ButtonBoard buttonBoard;
    private CommandButtonBoard commandButtonBoard;


    private GenericHIDSim sim;

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");

        /* Removes leftover state from previous tests - cancels commands,
        clears button triggers, and deregisters composed commands */
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().getDefaultButtonLoop().clear();
        CommandScheduler.getInstance().clearComposedCommands();

        //Simulate joystick connection with 13 buttons and 1 axis
        DriverStationSim.notifyNewData();
        DriverStationSim.setJoystickButtonCount(TEST_PORT, 13);
        DriverStationSim.setJoystickAxisCount(TEST_PORT, 1);

        //Enables simulated robot
        DriverStationSim.setEnabled(true);

        //Classes being tested
        buttonBoard = new ButtonBoard(TEST_PORT);
        commandButtonBoard = new CommandButtonBoard(buttonBoard);

        /* Creates a new simulation tied to buttonBoard which allows tests to programmatically press and release buttons.
        Then pushes initial state to Driver Station sim. */
        sim = new GenericHIDSim(buttonBoard);
        sim.notifyNewData();
    }

    @AfterEach
    void tearDown() {
        //Releases all 13 stimulate joystick buttons to unpressed
        for (int i = 1; i <= 13; i++) {
            sim.setRawButton(i, false);
        }
        sim.notifyNewData();

        //Reset Driver Station Metadata and reported button and axis counts on simulated joystick
        DriverStationSim.setJoystickButtonCount(TEST_PORT, 0);
        DriverStationSim.setJoystickAxisCount(TEST_PORT, 0);
        DriverStationSim.notifyNewData();

        /*Reset Command Scheduler - stops currently running command and
        removes the registry of composed commands so WPILib doesn't throw an error*/
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearComposedCommands();
    }


    //Enum tests
    @Test
    void buttonEnumValues_areCorrect() {
        assertEquals(2,  ButtonBoard.Button.kTopLeft.value);
        assertEquals(5,  ButtonBoard.Button.kTopCenter.value);
        assertEquals(12, ButtonBoard.Button.kTopRight.value);
        assertEquals(3,  ButtonBoard.Button.kMiddleLeft.value);
        assertEquals(4,  ButtonBoard.Button.kMiddleCenter.value);
        assertEquals(10, ButtonBoard.Button.kMiddleRight.value);
        assertEquals(9,  ButtonBoard.Button.kBottomLeft.value);
        assertEquals(11, ButtonBoard.Button.kBottomCenter.value);
        assertEquals(13, ButtonBoard.Button.kBottomRight.value);
    }

    //Makes sure that the toString() is doing cosmetic name formatting correctly
    @Test
    void buttonEnumToString_stripsLeadingK() {
        assertEquals("TopLeftButton",      ButtonBoard.Button.kTopLeft.toString());
        assertEquals("TopCenterButton",    ButtonBoard.Button.kTopCenter.toString());
        assertEquals("TopRightButton",     ButtonBoard.Button.kTopRight.toString());
        assertEquals("MiddleLeftButton",   ButtonBoard.Button.kMiddleLeft.toString());
        assertEquals("MiddleCenterButton", ButtonBoard.Button.kMiddleCenter.toString());
        assertEquals("MiddleRightButton",  ButtonBoard.Button.kMiddleRight.toString());
        assertEquals("BottomLeftButton",   ButtonBoard.Button.kBottomLeft.toString());
        assertEquals("BottomCenterButton", ButtonBoard.Button.kBottomCenter.toString());
        assertEquals("BottomRightButton",  ButtonBoard.Button.kBottomRight.toString());
    }

    //Held state tests
    @Test
    void getTopLeftButton_returnsTrueWhenPressed() {
        press(ButtonBoard.Button.kTopLeft.value);
        assertTrue(buttonBoard.getTopLeftButton());
    }

    @Test
    void getTopCenterButton_returnsTrueWhenPressed() {
        press(ButtonBoard.Button.kTopCenter.value);
        assertTrue(buttonBoard.getTopCenterButton());
    }

    @Test
    void getTopRightButton_returnsTrueWhenPressed() {
        press(ButtonBoard.Button.kTopRight.value);
        assertTrue(buttonBoard.getTopRightButton());
    }

    @Test
    void getMiddleLeftButton_returnsTrueWhenPressed() {
        press(ButtonBoard.Button.kMiddleLeft.value);
        assertTrue(buttonBoard.getMiddleLeftButton());
    }

    @Test
    void getMiddleCenterButton_returnsTrueWhenPressed() {
        press(ButtonBoard.Button.kMiddleCenter.value);
        assertTrue(buttonBoard.getMiddleCenterButton());
    }

    @Test
    void getMiddleRightButton_returnsTrueWhenPressed() {
        press(ButtonBoard.Button.kMiddleRight.value);
        assertTrue(buttonBoard.getMiddleRightButton());
    }

    @Test
    void getBottomLeftButton_returnsTrueWhenPressed() {
        press(ButtonBoard.Button.kBottomLeft.value);
        assertTrue(buttonBoard.getBottomLeftButton());
    }

    @Test
    void getBottomCenterButton_returnsTrueWhenPressed() {
        press(ButtonBoard.Button.kBottomCenter.value);
        assertTrue(buttonBoard.getBottomCenterButton());
    }

    @Test
    void getBottomRightButton_returnsTrueWhenPressed() {
        press(ButtonBoard.Button.kBottomRight.value);
        assertTrue(buttonBoard.getBottomRightButton());
    }

    @Test
    void allButtons_returnFalseWhenNotPressed() {
        assertFalse(buttonBoard.getTopLeftButton());
        assertFalse(buttonBoard.getTopCenterButton());
        assertFalse(buttonBoard.getTopRightButton());
        assertFalse(buttonBoard.getMiddleLeftButton());
        assertFalse(buttonBoard.getMiddleCenterButton());
        assertFalse(buttonBoard.getMiddleRightButton());
        assertFalse(buttonBoard.getBottomLeftButton());
        assertFalse(buttonBoard.getBottomCenterButton());
        assertFalse(buttonBoard.getBottomRightButton());
    }


    //Edge detection tests - makes sure that each physical transition of press and release is only read once
    @Test
    void getTopLeftButtonPressed_trueOnRisingEdge() {
        press(ButtonBoard.Button.kTopLeft.value);
        assertTrue(buttonBoard.getTopLeftButtonPressed(), "should be true on first call after press");
        assertFalse(buttonBoard.getTopLeftButtonPressed(), "should be false on second call (edge consumed)");
    }

    @Test
    void getTopLeftButtonReleased_trueOnFallingEdge() {
        press(ButtonBoard.Button.kTopLeft.value);
        buttonBoard.getTopLeftButtonPressed();

        release(ButtonBoard.Button.kTopLeft.value);
        assertTrue(buttonBoard.getTopLeftButtonReleased(), "should be true on release");
        assertFalse(buttonBoard.getTopLeftButtonReleased(), "should be false on second call (edge consumed)");
    }


    //Asserts the callback fired confirming the event triggers while the button is actively held
    @Test
    void topLeft_eventLoop_firesWhenButtonHeld() {
        EventLoop loop = new EventLoop();
        boolean[] fired = {false};

        buttonBoard.topLeft(loop).ifHigh(() -> fired[0] = true);
        press(ButtonBoard.Button.kTopLeft.value);
        loop.poll();

        assertTrue(fired[0], "topLeft BooleanEvent should fire while button is held");
    }

    //Asserts the callback didn't fire by polling the loop
    @Test
    void bottomRight_eventLoop_doesNotFireWhenReleased() {
        EventLoop loop = new EventLoop();
        boolean[] fired = {false};

        buttonBoard.bottomRight(loop).ifHigh(() -> fired[0] = true);
        loop.poll();

        assertFalse(fired[0], "bottomRight BooleanEvent should not fire when button is released");
    }

    //CommandButtonBoard tests
    //Verifies getHid() returns ButtonBoard instance not any GenericHID - CommandButtonBoard is holding the right HID object
    @Test
    void commandButtonBoard_getHID_returnsButtonBoardInstance() {
        assertInstanceOf(ButtonBoard.class, commandButtonBoard.getHID());
    }

    /*Very the CommandButtonBoard triggers correctly with WPILib's command system - binds an InstantCommand to fire onTrue (rising edge),
     presses button, evaluates trigger condition, runs the scheduler to execute any scheduled commands and asserted the command ran
     */
    @Test
    void commandButtonBoard_topLeft_triggerActivatesWhenButtonPressed() {
        Trigger trigger = commandButtonBoard.topLeft();
        boolean[] ran = {false};

        trigger.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> ran[0] = true));

        press(ButtonBoard.Button.kTopLeft.value);

        CommandScheduler.getInstance().getDefaultButtonLoop().poll();
        CommandScheduler.getInstance().run();

        assertTrue(ran[0], "topLeft Trigger should schedule its command on press");
    }

    //Does the same thing with different button on the button board
    @Test
    void commandButtonBoard_bottomRight_triggerActivatesWhenButtonPressed() {
        Trigger trigger = commandButtonBoard.bottomRight();
        boolean[] ran = {false};

        trigger.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> ran[0] = true));

        press(ButtonBoard.Button.kBottomRight.value);

        CommandScheduler.getInstance().getDefaultButtonLoop().poll();
        CommandScheduler.getInstance().run();

        assertTrue(ran[0], "bottomRight Trigger should schedule its command on press");
    }

    //Confirms with no button presses the trigger's current state returns false
    @Test
    void commandButtonBoard_middleCenter_triggerInactiveWhenNotPressed() {
        Trigger trigger = commandButtonBoard.middleCenter();
        assertFalse(trigger.getAsBoolean());
    }

    //Each sets the button state on the sim and pushes it to the driver station through notifyNewData
    private void press(int buttonNumber) {
        sim.setRawButton(buttonNumber, true);
        sim.notifyNewData();
    }

    private void release(int buttonNumber) {
        sim.setRawButton(buttonNumber, false);
        sim.notifyNewData();
    }
}
