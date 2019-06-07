package org.firstinspires.ftc.teamcode.Robot;

import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.io.File;
import java.io.FileOutputStream;

@Disabled
@TeleOp (name = "VuforiaImageRippingTest")
public class VuforiaToOpenCV extends LinearOpMode implements Runnable{

    private static Mat lastImage;

    private VuforiaLocalizer vuforia;

    public VuforiaToOpenCV(){

    }

    public VuforiaToOpenCV(VuforiaLocalizer vuforia){
        this.vuforia=vuforia;
    }

    private void initVuforia() {
        final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.FRONT;
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AWRj/xj/////AAAAGQxUvYmWcEwclJxNRPrJOqYGevnetxTRA6P/gYZfb+gZsAeO0GosrfKmnO2O24hVPv8v1YQYA8vQ5qc1eVjwOjZPCykT4eRXxqxLZV/7BJmEraEs991INaYI9qXQjGkbeWbLQT/e7zJAvxsRVjQjRPukDLapC4dmdA5YbXvxy9pR2+LokoO6PiSLl9ktBte3BFGHQepiugBC7C1jXDfkClwTb/+R7OwaVuL1gp6rWun5Cn42RHysv4HsTkBMShaKdL4/whXVRmrYfkMMsAtihEAK+rLs8fWnmVB1Z/UJ67QIWqP04Va/u/mbTErjPDiRvCnYhGmIWTpIt+9slhip9vT9pRfLIe+gcfAIajoF6wUe";
        parameters.cameraDirection = CAMERA_CHOICE;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        vuforia.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
    }

    public Mat getLastImage(){
        return lastImage;
    }

    public Image getImageFromFrame() {
        try {
            VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
            long numImgs = frame.getNumImages();
            for (int i = 0; i < numImgs; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    Log.v("Image Processing", "Image taking successful");
                    return frame.getImage(i);
                }
            }
        } catch (Exception e) {
            Log.w("Image Processing", "Exception- Image taking failed");
        }
        return null;
    }

    public static Bitmap imageToBitmap(Image image){
        try {
            Bitmap bmp;
            bmp = Bitmap.createBitmap(image.getWidth(),image.getHeight(), Bitmap.Config.RGB_565);
//            bmp = Bitmap.createBitmap(null,null,0,image.getStride(),image.getWidth(),image.getHeight(), Bitmap.Config.RGB_565);
            bmp.copyPixelsFromBuffer(image.getPixels());
            Log.v("Image Processing", "Bitmap conversion successful");
            return bmp;
        } catch (Exception e){
            Log.w("Image Processing", "Exception- Bitmap conversion failed");
        }
        return null;
    }

    public Mat getMatFromVuforia(){
        try {
            Mat mat = new Mat();
            Bitmap bmp = imageToBitmap(getImageFromFrame());
            Utils.bitmapToMat(bmp, mat);
            return mat;
        } catch (Exception e){
            Log.w("Image Processing","Exception- Mat conversion failed");
        }
        return null;
    }

    public void saveMatImage(Mat mat, String name){
        Bitmap bitmapFromMat = Bitmap.createBitmap(mat.width(),mat.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(mat,bitmapFromMat);


        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        File matFile = new File(path, name+".png");
        String matFilepath = matFile.toString();

        try {
            FileOutputStream matOut = new FileOutputStream(matFilepath);
            bitmapFromMat.compress(Bitmap.CompressFormat.PNG, 100, matOut);
        } catch (Exception e){
            Log.w("Image Processing","Exception- Image saving failed");
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initVuforia();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            Mat mat = getMatFromVuforia();
            saveMatImage(mat, "matOutput");
            sleep(100);
        }
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted() && !isStopRequested()){
            lastImage=getMatFromVuforia();
            sleep (100);
        }
    }
}