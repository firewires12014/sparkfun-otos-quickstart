package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Gamepad Tester", group = "Test")
public class gamepadTester extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Gamepad 1
        telemetry.addLine("=== Gamepad 1 ===");
        telemetry.addData("Left Stick (x,y)", "%.3f, %.3f", gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.addData("Right Stick (x,y)", "%.3f, %.3f", gamepad1.right_stick_x, gamepad1.right_stick_y);
        telemetry.addData("Left Stick Btn", booleanStr(gamepad1.left_stick_button));
        telemetry.addData("Right Stick Btn", booleanStr(gamepad1.right_stick_button));
        telemetry.addData("Triggers (left,right)", "%.3f, %.3f", gamepad1.left_trigger, gamepad1.right_trigger);
        telemetry.addData("Bumpers (LB,RB)", "%b, %b", gamepad1.left_bumper, gamepad1.right_bumper);
        telemetry.addData("Buttons (A,B,X,Y)", "%b, %b, %b, %b", gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y);
        telemetry.addData("DPad (up,down,left,right)", "%b, %b, %b, %b", gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left, gamepad1.dpad_right);
        telemetry.addData("Start/Back", "%b, %b", gamepad1.start, gamepad1.back);

        telemetry.addLine("");

        // Gamepad 2
        telemetry.addLine("=== Gamepad 2 ===");
        telemetry.addData("Left Stick (x,y)", "%.3f, %.3f", gamepad2.left_stick_x, gamepad2.left_stick_y);
        telemetry.addData("Right Stick (x,y)", "%.3f, %.3f", gamepad2.right_stick_x, gamepad2.right_stick_y);
        telemetry.addData("Left Stick Btn", booleanStr(gamepad2.left_stick_button));
        telemetry.addData("Right Stick Btn", booleanStr(gamepad2.right_stick_button));
        telemetry.addData("Triggers (left,right)", "%.3f, %.3f", gamepad2.left_trigger, gamepad2.right_trigger);
        telemetry.addData("Bumpers (LB,RB)", "%b, %b", gamepad2.left_bumper, gamepad2.right_bumper);
        telemetry.addData("Buttons (A,B,X,Y)", "%b, %b, %b, %b", gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y);
        telemetry.addData("DPad (up,down,left,right)", "%b, %b, %b, %b", gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.dpad_left, gamepad2.dpad_right);
        telemetry.addData("Start/Back", "%b, %b", gamepad2.start, gamepad2.back);

        telemetry.addLine("");
        telemetry.addData("Runtime (s)", "%.3f", getRuntime());

        telemetry.update();
    }

    private String booleanStr(boolean b) {
        return b ? "PRESSED" : "released";
    }
}
