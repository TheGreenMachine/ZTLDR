package com.team1816.lib.inputs;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.function.FloatConsumer;
import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.event.EventLoop;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static org.junit.jupiter.api.Assertions.*;

public class ButtonBoardTest {

    private ButtonBoard buttonBoard;

    @BeforeEach
    public void testInit() {
        buttonBoard = new ButtonBoard(2);
    }

    // -------------------------------------------------------------------------
    // Button enum – raw ID values
    // -------------------------------------------------------------------------

    @Test
    public void testButtonEnumValues() {
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

    // -------------------------------------------------------------------------
    // Button enum – toString strips leading 'k'
    // -------------------------------------------------------------------------

    @Test
    public void testButtonEnumToString() {
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

    // -------------------------------------------------------------------------
    // Button enum – all 9 values are unique
    // -------------------------------------------------------------------------

    @Test
    public void testButtonEnumValuesAreUnique() {
        ButtonBoard.Button[] buttons = ButtonBoard.Button.values();
        for (int i = 0; i < buttons.length; i++) {
            for (int j = i + 1; j < buttons.length; j++) {
                assertNotEquals(
                    buttons[i].value,
                    buttons[j].value,
                    buttons[i] + " and " + buttons[j] + " share the same raw ID"
                );
            }
        }
    }

    // -------------------------------------------------------------------------
    // Button enum – enum count is exactly 9
    // -------------------------------------------------------------------------

    @Test
    public void testButtonEnumCount() {
        assertEquals(9, ButtonBoard.Button.values().length);
    }

    // -------------------------------------------------------------------------
    // EventLoop methods return non-null BooleanEvents
    // -------------------------------------------------------------------------

    @Test
    public void testEventLoopMethodsReturnNonNull() {
        EventLoop loop = new EventLoop();
        assertNotNull(buttonBoard.topLeft(loop));
        assertNotNull(buttonBoard.topCenter(loop));
        assertNotNull(buttonBoard.topRight(loop));
        assertNotNull(buttonBoard.middleLeft(loop));
        assertNotNull(buttonBoard.middleCenter(loop));
        assertNotNull(buttonBoard.middleRight(loop));
        assertNotNull(buttonBoard.bottomLeft(loop));
        assertNotNull(buttonBoard.bottomCenter(loop));
        assertNotNull(buttonBoard.bottomRight(loop));
    }

    // -------------------------------------------------------------------------
    // Each EventLoop method binds to the correct button (distinct events)
    // -------------------------------------------------------------------------

    @Test
    public void testEventLoopMethodsReturnDistinctEvents() {
        EventLoop loop = new EventLoop();
        assertNotSame(buttonBoard.topLeft(loop),    buttonBoard.topRight(loop));
        assertNotSame(buttonBoard.middleLeft(loop), buttonBoard.middleRight(loop));
        assertNotSame(buttonBoard.bottomLeft(loop), buttonBoard.bottomRight(loop));
    }

    // -------------------------------------------------------------------------
    // Sendable – initSendable registers correct type and all 9 properties
    // -------------------------------------------------------------------------

    @Test
    public void testInitSendableRegistersControllerType() {
        // Capture what type string gets registered via a minimal hand-rolled builder.
        String[] capturedType = {null};
        String[] capturedControllerType = {null};
        int[] booleanPropertyCount = {0};

        SendableBuilder builder = new SendableBuilder() {
            @Override
            public void close() throws Exception {

            }

            @Override public void setSmartDashboardType(String type) { capturedType[0] = type; }
            @Override public void publishConstString(String key, String value) {
                if ("ControllerType".equals(key)) capturedControllerType[0] = value;
            }

            @Override
            public void addBooleanArrayProperty(String key, Supplier<boolean[]> getter, Consumer<boolean[]> setter) {

            }

            @Override
            public void publishConstBooleanArray(String key, boolean[] value) {

            }

            @Override
            public void addIntegerArrayProperty(String key, Supplier<long[]> getter, Consumer<long[]> setter) {

            }

            @Override
            public void publishConstIntegerArray(String key, long[] value) {

            }

            @Override
            public void addFloatArrayProperty(String key, Supplier<float[]> getter, Consumer<float[]> setter) {

            }

            @Override
            public void publishConstFloatArray(String key, float[] value) {

            }

            @Override
            public void addDoubleArrayProperty(String key, Supplier<double[]> getter, Consumer<double[]> setter) {

            }

            @Override
            public void publishConstDoubleArray(String key, double[] value) {

            }

            @Override
            public void addStringArrayProperty(String key, Supplier<String[]> getter, Consumer<String[]> setter) {

            }

            @Override
            public void publishConstStringArray(String key, String[] value) {

            }

            public void addBooleanProperty(String key, java.util.function.BooleanSupplier getter, java.util.function.Consumer<Boolean> setter) {
                booleanPropertyCount[0]++;
            }
            // remaining interface methods are no-ops
            @Override public void setActuator(boolean value) {}
            @Override public void setSafeState(Runnable func) {}

            @Override
            public void addBooleanProperty(String key, BooleanSupplier getter, BooleanConsumer setter) {

            }

            @Override public void addDoubleProperty(String key, java.util.function.DoubleSupplier getter, java.util.function.DoubleConsumer setter) {}
            @Override public void publishConstDouble(String key, double value) {}
            @Override public void publishConstBoolean(String key, boolean value) {}
            @Override public void publishConstInteger(String key, long value) {}

            @Override
            public void addFloatProperty(String key, FloatSupplier getter, FloatConsumer setter) {

            }

            @Override
            public void publishConstFloat(String key, float value) {

            }

            @Override public void addStringProperty(String key, java.util.function.Supplier<String> getter, java.util.function.Consumer<String> setter) {}
            @Override public void addIntegerProperty(String key, java.util.function.LongSupplier getter, java.util.function.LongConsumer setter) {}
            public void addFloatProperty(String key, java.util.function.Supplier<Float> getter, java.util.function.Consumer<Float> setter) {}
            @Override public void addRawProperty(String key, String typeString, java.util.function.Supplier<byte[]> getter, java.util.function.Consumer<byte[]> setter) {}

            @Override
            public void publishConstRaw(String key, String typeString, byte[] value) {

            }

            @Override
            public BackendKind getBackendKind() {
                return null;
            }

            @Override
            public boolean isPublished() {
                return false;
            }

            @Override public void update() {}
            @Override public void clearProperties() {}
            @Override public void addCloseable(AutoCloseable closeable) {}
        };

        assertDoesNotThrow(() -> buttonBoard.initSendable(builder));
        assertEquals("HID",         capturedType[0]);
        assertEquals("ButtonBoard",  capturedControllerType[0]);
        assertEquals(9,              booleanPropertyCount[0]);
    }
}
