package org.firstinspires.ftc.teamcode.Robot;

import android.app.Activity;
import android.content.ActivityNotFoundException;
import android.content.Context;
import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.Callable;

/**
 * Created by August on 1/13/2018.
 */

public class CubeDetector implements Runnable {
    public List<MatOfPoint> contours;
    private Mat hierarchy;
    private Mat inputMat;
    OpMode opMode;
    Activity activity;
    ImageView outputImage;

    final double topCropProportion = 0.2;
    final double bottomCropProportion = 0.45;

    public CubeDetector(OpMode opMode){
        this.opMode=opMode;
        this.activity = (Activity) opMode.hardwareMap.appContext;
        this.outputImage = (ImageView) activity.findViewById(R.id.openCVOutput);

    }

    public boolean isThreadRunning() {
        return threadRunning;
    }

    private boolean threadRunning = false;


//    // Brown cubes
//    Scalar minValues = new Scalar(102, 40, 40);
//
//    Scalar maxValues = new Scalar(150, 255, 220);

    Scalar minValues = new Scalar(0, 0, 60);

    Scalar maxValues = new Scalar(255, 100, 255);


    public void loadImage(VuforiaToOpenCV matGetter) {
        Mat image;
        do {
            image = matGetter.getMatFromVuforia();
        } while (image == null);

        setInputMat(image);
        new Thread(this).start();
    }

    public void loadImageUsingThread(VuforiaToOpenCV matGetter) {
        threadRunning = true;
        Mat image;
        do {
            image = matGetter.getLastImage();
        } while (image == null);

        setInputMat(image);
        new Thread(this).start();
    }


    public Mat getInputMat() {
        return inputMat.clone();
    }

    public Rect getBestRect() {
        Rect bestRect = new Rect(0, 0, 0, 0);
        for (Rect rect : getRects()) {
            if (rect.area() > bestRect.area()) {
                bestRect = rect;
            }
        }
        return bestRect;
    }

    public void setInputMat(Mat inputMat) {
        this.inputMat = inputMat.clone();
    }

    public int calcCrabDistance() {
        Rect bestRect = getBestRect();
        if (bestRect.x == 0) {
            return 0;
        }
        double cubePos = bestRect.x + (bestRect.width/2.0);
        return (int) (4 * (cubePos - (inputMat.width()/2.0)));
    }

    public void calculateContours(Mat input) {
        Mat croppedImage;
        Mat blurredImage = new Mat();
        Mat hsvImage = new Mat();
        Mat mask = new Mat();
        Mat morphOutput;
        contours = new ArrayList<>();
        hierarchy = new Mat();

        croppedImage = new Mat(input, new Rect(0, (int) (input.height() * topCropProportion), input.width(), (int) (input.height() * ((1 - topCropProportion) - bottomCropProportion))));

        // remove some noise
        Imgproc.blur(croppedImage, blurredImage, new Size(5, 5));

        // convert the frame to HSV
        Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);


        // show the current selected HSV range
        Core.inRange(hsvImage, minValues, maxValues, mask);
//        Core.inRange(blurredImage, minValues, maxValues, mask);

        // morphological operator
        // dilate with large element, erode with small ones
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10, 10));
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(60, 60));


        morphOutput = mask.clone();

        Imgproc.dilate(morphOutput, morphOutput, dilateElement);
        Imgproc.erode(morphOutput, morphOutput, erodeElement);

        Imgproc.dilate(morphOutput, morphOutput, dilateElement);
        Imgproc.erode(morphOutput, morphOutput, erodeElement);


        // find contours
        Imgproc.findContours(morphOutput, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

    }

    public void updateScreen(){
        Mat contourMat = new Mat(inputMat, new Rect(0, (int) (inputMat.height() * topCropProportion), inputMat.width(), (int) (inputMat.height() * ((1 - topCropProportion) - bottomCropProportion))));


        if (hierarchy.size().height > 0 && hierarchy.size().width > 0) {
            // for each contour, display it in blue
            for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0])
            {
                Imgproc.drawContours(contourMat, contours, idx, new Scalar(255, 0, 0),5);
                System.out.println(Arrays.toString(contours.get(idx).toArray()));
                try {
                    Rect rect = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(idx).toArray())).boundingRect();

                    Imgproc.rectangle(contourMat, new Point(rect.x,rect.y),new Point(rect.x+rect.width,rect.y+rect.height), new Scalar(0,0,255),10);
                } catch (Exception e){

                }
                Rect bestRect = getBestRect();
                Imgproc.rectangle(contourMat, new Point(bestRect.x,bestRect.y),new Point(bestRect.x+bestRect.width,bestRect.y+bestRect.height), new Scalar(0,255,0),10);

            }
        }
        Bitmap contourBitmap = Bitmap.createBitmap(contourMat.width(),contourMat.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(contourMat,contourBitmap);


        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        File matFile = new File(path, "jdsljfsad"+".png");
        String matFilepath = matFile.toString();

        try {
            FileOutputStream matOut = new FileOutputStream(matFilepath);
            contourBitmap.compress(Bitmap.CompressFormat.PNG, 100, matOut);
        } catch (Exception e){
            Log.w("Image Processing","Exception- Image saving failed");
        }
        try {
            outputImage.setImageBitmap(contourBitmap);
        } catch (Exception e){
            Log.e("Image Processing", e.getMessage());
        }
    }

    public ArrayList<Rect> getRects() {
        ArrayList<Rect> rects = new ArrayList<>();
        if (hierarchy.size().height > 0 && hierarchy.size().width > 0) {
            for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0]) {
                rects.add(Imgproc.minAreaRect(new MatOfPoint2f(contours.get(idx).toArray())).boundingRect());
            }
        }
        return rects;
    }

    @Override
    public void run() {
        threadRunning = true;
        calculateContours(inputMat);
        threadRunning = false;
    }
}
