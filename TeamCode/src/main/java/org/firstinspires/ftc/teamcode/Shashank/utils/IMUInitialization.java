package org.firstinspires.ftc.teamcode.Shashank.utils;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;

/**
 * Created by spmeg on 11/5/2016.
 */

public class IMUInitialization {
    private HardwareMap hardwareMap;
    private BNO055IMU imu;

    public IMUInitialization(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public BNO055IMU getIMU(){
        BufferedReader reader = null;
        try {
            reader = new BufferedReader(new InputStreamReader(hardwareMap.appContext.openFileInput("AdafruitIMUCalibration.json")));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        BNO055IMU.CalibrationData calibrationData = new BNO055IMU.CalibrationData();
        JsonParser jsonParser = new JsonParser();
        try {
            String line = reader.readLine();
            JsonElement element = jsonParser.parse(line);
            if(element.isJsonObject()){
                JsonObject object = element.getAsJsonObject();
                JsonElement dxAccelJson = object.get("dxAccel");
                calibrationData.dxAccel = dxAccelJson.getAsShort();

                JsonElement dyAccelJson = object.get("dyAccel");
                calibrationData.dyAccel = dyAccelJson.getAsShort();

                JsonElement dzAccelJson = object.get("dzAccel");
                calibrationData.dzAccel = dzAccelJson.getAsShort();

                JsonElement dxGyroJson = object.get("dxGyro");
                calibrationData.dxGyro = dxGyroJson.getAsShort();

                JsonElement dyGyroJson = object.get("dyGyro");
                calibrationData.dyGyro = dyGyroJson.getAsShort();

                JsonElement dzGyroJson = object.get("dzGyro");
                calibrationData.dzGyro = dzGyroJson.getAsShort();

                JsonElement dxMagJson = object.get("dxMag");
                calibrationData.dxMag = dxMagJson.getAsShort();

                JsonElement dyMagJson = object.get("dyMag");
                calibrationData.dyMag = dyMagJson.getAsShort();

                JsonElement dzMagJson = object.get("dzMag");
                calibrationData.dzMag = dzMagJson.getAsShort();

                JsonElement radiusAccelJson = object.get("radiusAccel");
                calibrationData.radiusAccel = radiusAccelJson.getAsShort();

                JsonElement radiusMagJson = object.get("radiusMag");
                calibrationData.radiusMag = radiusMagJson.getAsShort();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationData     = calibrationData;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        return imu;
    }
}
