package frc.robot.systems.util;

public final class T16000M {

    private T16000M() {};

    public enum Button{

        TRIGGER(1),

        GRIP_BOTTOM(2),
        GRIP_LEFT(3),
        GRIP_RIGHT(4),

        RIGHT_TOP_RIGHT(5),
        RIGHT_TOP_CENTER(6),
        RIGHT_TOP_LEFT(7),

        RIGHT_BOTTOM_LEFT(8),
        RIGHT_BOTTOM_CENTER(9),
        RIGHT_BOTTOM_RIGHT(10),

        LEFT_TOP_LEFT(11),
        LEFT_TOP_CENTER(12),
        LEFT_TOP_RIGHT(13),

        LEFT_BOTTOM_RIGHT(14),
        LEFT_BOTTOM_CENTER(15),
        LEFT_BOTTOM_LEFT(16);
        
        private final int buttonNumber;

        Button(int buttonNumber){
            this.buttonNumber = buttonNumber;
        }
    }

    public enum Axis{
        X(0),
        Y(1),
        TWIST(2),
        THROTTLE(3);

        private final int axisNumber;

        Axis(int axisNumber){
            this.axisNumber = axisNumber;
        }
    }
}
