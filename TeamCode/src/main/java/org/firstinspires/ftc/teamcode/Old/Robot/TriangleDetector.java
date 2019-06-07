package org.firstinspires.ftc.teamcode.Robot;

import android.app.Activity;
import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.opengl.models.Geometry;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Created by August on 3/15/2018.
 */

public class TriangleDetector {
    public List<MatOfPoint> contours;
    private Mat hierarchy;
    private Mat inputMat;
    OpMode opMode;
    Activity activity;
    ImageView outputImage;

    final double topCropProportion = 0.60;
    final double bottomCropProportion = 0.30;

    Scalar blueMinValues = new Scalar(0, 120, 0);
    Scalar blueMaxValues = new Scalar(90, 255, 255);

    Scalar redMinValues = new Scalar(110, 120, 0);
    Scalar redMaxValues = new Scalar(180, 255, 255);

    Scalar minValues;
    Scalar maxValues;

    public Mat getInputMat() {
        return inputMat.clone();
    }

    public int calculateCrabDistance(){
        return (int)(15*(calculateCenter()-(getInputMat().width()/2.0)));
    }

    public void setInputMat(Mat inputMat) {
        this.inputMat = inputMat.clone();
    }

    public void loadImage(VuforiaToOpenCV matGetter) {
        Mat image;
        do {
            image = matGetter.getMatFromVuforia();
        } while (image == null);

        setInputMat(image);
    }

    public TriangleDetector(OpMode opMode, TeamColor teamColor){
        this.opMode=opMode;
        this.activity = (Activity) opMode.hardwareMap.appContext;
        this.outputImage = (ImageView) activity.findViewById(R.id.openCVOutput);
        if (teamColor==teamColor.RED){
            minValues=redMinValues;
            maxValues=redMaxValues;
        } else {
            minValues=blueMinValues;
            maxValues=blueMaxValues;
        }
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

        Core.normalize(croppedImage,croppedImage,255,0,Core.NORM_INF);

        // remove some noise
        Imgproc.blur(croppedImage, blurredImage, new Size(5, 5));

        // convert the frame to HSV
        Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);


        // show the current selected HSV range
        Core.inRange(hsvImage, minValues, maxValues, mask);
//        Core.inRange(blurredImage, minValues, maxValues, mask);

        // morphological operator
        // dilate with large element, erode with small ones
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(20, 20));
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10, 10));


        morphOutput = mask.clone();

        Imgproc.erode(morphOutput, morphOutput, erodeElement);
        Imgproc.dilate(morphOutput, morphOutput, dilateElement);

        Imgproc.erode(morphOutput, morphOutput, erodeElement);
        Imgproc.dilate(morphOutput, morphOutput, dilateElement);


        // find contours
        Imgproc.findContours(morphOutput, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

    }


    double calculateCenter(){
        List<Rect> rects = getRects();

        if (rects.size()>=2) {
            return ((rects.get(0).x + (rects.get(0).width / 2.0))
                    + (rects.get(1).x + (rects.get(1).width / 2.0)))
                    / 2.0;
        }
        else return getInputMat().width()/2;
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
                Imgproc.circle(contourMat, new Point(calculateCenter(),contourMat.height()/2),30, new Scalar(0,255,0),10);

            }
        }
        Bitmap contourBitmap = Bitmap.createBitmap(contourMat.width(),contourMat.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(contourMat,contourBitmap);


//        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
//        File matFile = new File(path, "jdsljfsad"+".png");
//        String matFilepath = matFile.toString();
//
//        try {
//            FileOutputStream matOut = new FileOutputStream(matFilepath);
//            contourBitmap.compress(Bitmap.CompressFormat.PNG, 100, matOut);
//        } catch (Exception e){
//            Log.w("Image Processing","Exception- Image saving failed");
//        }
        try {
            outputImage.setImageBitmap(contourBitmap);
        } catch (Exception e){
            Log.e("Image Processing", e.getMessage());
        }
    }

    public List<Rect> getRects() {
        List<Rect> rects = new ArrayList<>();
        if (hierarchy.size().height > 0 && hierarchy.size().width > 0) {
            for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0]) {
                rects.add(Imgproc.minAreaRect(new MatOfPoint2f(contours.get(idx).toArray())).boundingRect());
            }
        }
        return rects;
    }
}
