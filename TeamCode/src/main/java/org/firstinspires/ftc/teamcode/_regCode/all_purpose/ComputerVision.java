package org.firstinspires.ftc.teamcode._regCode.all_purpose;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.content.Context;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

/**
 * This class creates the basis for object detection
 */
public class ComputerVision {

    public HardwareMap hardwareMap;

    //------------------------------------------------------------------------------------------------
    // Computer Vision
    //------------------------------------------------------------------------------------------------
    public final WebcamName Camera;

    public void StartCamView(){
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

        camera.setPipeline(new SampleDetector());

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

        //camera.setPipeline(yourPipeline);

    }

    public ComputerVision(WebcamName currentCamera, HardwareMap hardwareMap){
        Camera = currentCamera;
        this.hardwareMap = hardwareMap;
    }
}
