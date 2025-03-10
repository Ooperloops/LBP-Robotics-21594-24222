package org.firstinspires.ftc.teamcode._regCode.human_operated;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode._regCode.all_purpose.AIGeneratedSampleDetector;
import org.firstinspires.ftc.teamcode._regCode.all_purpose.ComputerVision;
import org.firstinspires.ftc.teamcode._regCode.all_purpose.HardwareManager;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


@TeleOp(name = "Camera Test", group = "TeleOp")
public class CameraTest extends LinearOpMode {

    private HardwareManager hardwareManager;
    private WebcamName Camera;

    @Override
    public void runOpMode() {
        hardwareManager = new HardwareManager(hardwareMap);
        Camera = hardwareManager.camera;
        int cameraMonitorViewId =
                hardwareMap.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName());

        // With live preview
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(
                Camera,
                cameraMonitorViewId
        );
        OpenCvPipeline pipeline = new AIGeneratedSampleDetector(hardwareManager.angleClawServo);
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Detected Angle", ((AIGeneratedSampleDetector)pipeline).getRotationAngle());
            telemetry.update();
        }
    }
}