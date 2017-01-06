package org.firstinspires.ftc.teamcode.Shashank.utils;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.StringTokenizer;

/**
 * Created by spmeg on 1/5/2017.
 */

public class ThreadSharedObject {
    private volatile HashMap<String, Integer> intSensorValues = new HashMap<>();
    private volatile HashMap<String, Double> doubleSensorValues = new HashMap<>();
    private volatile HashMap<String, Boolean> booleanSensorValues = new HashMap<>();
    private volatile HashMap<String, String> stringSensorValues = new HashMap<>();

    public int getInteger(String tag){
        return intSensorValues.get(tag);
    }

    public double getDouble(String tag){
        return doubleSensorValues.get(tag);
    }

    public boolean getBoolean(String tag){
        return booleanSensorValues.get(tag);
    }

    public String getString(String tag){
        return stringSensorValues.get(tag);
    }

    public synchronized void setInteger(String tag, int value){
        intSensorValues.put(tag, value);
    }

    public synchronized void setDouble(String tag, double value){
        doubleSensorValues.put(tag, value);
    }

    public synchronized void setBoolean(String tag, boolean value){
        booleanSensorValues.put(tag, value);
    }

    public synchronized void setString(String tag, String value){
        stringSensorValues.put(tag, value);
    }

    public HashMap<String, Integer> getIntSensorValues() {
        return intSensorValues;
    }

    public void setIntSensorValues(HashMap<String, Integer> intSensorValues) {
        this.intSensorValues = intSensorValues;
    }

    public HashMap<String, Double> getDoubleSensorValues() {
        return doubleSensorValues;
    }

    public void setDoubleSensorValues(HashMap<String, Double> doubleSensorValues) {
        this.doubleSensorValues = doubleSensorValues;
    }

    public HashMap<String, Boolean> getBooleanSensorValues() {
        return booleanSensorValues;
    }

    public void setBooleanSensorValues(HashMap<String, Boolean> booleanSensorValues) {
        this.booleanSensorValues = booleanSensorValues;
    }

    public HashMap<String, String> getStringSensorValues() {
        return stringSensorValues;
    }

    public void setStringSensorValues(HashMap<String, String> stringSensorValues) {
        this.stringSensorValues = stringSensorValues;
    }
}
