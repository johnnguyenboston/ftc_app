//package org.firstinspires.ftc.teamcode.Robot;
//
//import com.qualcomm.robotcore.hardware.HardwareDevice;
//import com.qualcomm.robotcore.hardware.I2cAddr;
//import com.qualcomm.robotcore.hardware.I2cController;
//import com.qualcomm.robotcore.hardware.I2cDevice;
//
//import java.util.concurrent.locks.Lock;
//
///**
// * Created by August on 1/28/2017.
// */
//
//public class PixySensor implements I2cDevice{
//    final int PIXY_START_WORD = 0xaa55;
//    final int PIXY_START_WORD_CC =  0XAA56;
//    final int PIXY_START_WORDX = 0x55aa;
//    final int PIXY_ARRAYSIZE = 100;
//
//    enum BlockType{
//        NORMAL_BLOCK,
//        CC_BLOCK
//    }
//
//    class Block{
//        int signature;
//        int x;
//        int y;
//        int width;
//        int hight;
//        int angle;
//    }
//
//    int getBlocks(int maxBlocks){
//        int i, w, blockCount, checksum, sum;
//        Block block;
//        if (g_skipStart==0){
//            if (getStart()==0){
//                return 0;
//            }
//        } else {
//            g_skipStart=0;
//        }
//        for (blockCount=0; blockCount<maxBlocks && blockCount<PIXY_ARRAYSIZE;){
//            checksum = getWord();
//            if (checksum==PIXY_START_WORD){
//                g_skipStart = 1;
//                g_blockType = BlockType.NORMAL_BLOCK;
//                return blockCount;
//            } else if (checksum==PIXY_START_WORD_CC){
//                g_skipStart=1;
//                g_blockType=BlockType.CC_BLOCK;
//                return blockCount;
//            } else if (checksum == 0){
//                return blockCount;
//            }
//        //    block = g_blocks + blockCount;
//        }
//    }
//
//    static int g_skipStart = 0;
//    static Block g_blocks;
//
//    public int getStart(){
//        int w, lastw;
//        lastw = 0xffff;
//        while (true){
//            w=getWord();
//            if (w==0 && lastw==0){
//                return 0;
//            } else if (w==PIXY_START_WORD && lastw==PIXY_START_WORD) {
//                g_blockType = BlockType.CC_BLOCK;
//                return 1;
//            } else if (w==PIXY_START_WORD) {
//                getByte();
//            }
//            lastw = w;
//        }
//    }
//
//    int getWord(){
//        int w;
//        int c;
//        c=getByte();
//        w=getByte();
//        w <<= 8;
//        w |= c;
//        return w;
//    }
//
//    int getByte();
//
//    static BlockType g_blockType;
//
//
//
//    @Override
//    public void enableI2cReadMode(I2cAddr i2cAddr, int register, int count) {
//
//    }
//
//    @Override
//    public void enableI2cWriteMode(I2cAddr i2cAddr, int register, int count) {
//
//    }
//
//    @Override
//    public boolean isI2cPortInReadMode() {
//        return false;
//    }
//
//    @Override
//    public boolean isI2cPortInWriteMode() {
//        return false;
//    }
//
//    @Override
//    public void readI2cCacheFromController() {
//
//    }
//
//    @Override
//    public void writeI2cCacheToController() {
//
//    }
//
//    @Override
//    public void writeI2cPortFlagOnlyToController() {
//
//    }
//
//    @Override
//    public void setI2cPortActionFlag() {
//
//    }
//
//    @Override
//    public boolean isI2cPortActionFlagSet() {
//        return false;
//    }
//
//    @Override
//    public void clearI2cPortActionFlag() {
//
//    }
//
//    @Override
//    public byte[] getI2cReadCache() {
//        return new byte[0];
//    }
//
//    @Override
//    public Lock getI2cReadCacheLock() {
//        return null;
//    }
//
//    @Override
//    public byte[] getI2cWriteCache() {
//        return new byte[0];
//    }
//
//    @Override
//    public Lock getI2cWriteCacheLock() {
//        return null;
//    }
//
//    @Override
//    public byte[] getCopyOfReadBuffer() {
//        return new byte[0];
//    }
//
//    @Override
//    public byte[] getCopyOfWriteBuffer() {
//        return new byte[0];
//    }
//
//    @Override
//    public void copyBufferIntoWriteBuffer(byte[] buffer) {
//
//    }
//
//    @Override
//    public void registerForI2cPortReadyCallback(I2cController.I2cPortReadyCallback callback) {
//
//    }
//
//    @Override
//    public I2cController.I2cPortReadyCallback getI2cPortReadyCallback() {
//        return null;
//    }
//
//    @Override
//    public void deregisterForPortReadyCallback() {
//
//    }
//
//    @Override
//    public int getCallbackCount() {
//        return 0;
//    }
//
//    @Override
//    public boolean isI2cPortReady() {
//        return false;
//    }
//
//    @Override
//    public void registerForPortReadyBeginEndCallback(I2cController.I2cPortReadyBeginEndNotifications callback) {
//
//    }
//
//    @Override
//    public I2cController.I2cPortReadyBeginEndNotifications getPortReadyBeginEndCallback() {
//        return null;
//    }
//
//    @Override
//    public void deregisterForPortReadyBeginEndCallback() {
//
//    }
//
//    @Override
//    public boolean isArmed() {
//        return false;
//    }
//
//    @Override
//    public I2cController getController() {
//        return null;
//    }
//
//    @Override
//    public void readI2cCacheFromModule() {
//
//    }
//
//    @Override
//    public void writeI2cCacheToModule() {
//
//    }
//
//    @Override
//    public void writeI2cPortFlagOnlyToModule() {
//
//    }
//
//    @Override
//    public Manufacturer getManufacturer() {
//        return null;
//    }
//
//    @Override
//    public String getDeviceName() {
//        return null;
//    }
//
//    @Override
//    public String getConnectionInfo() {
//        return null;
//    }
//
//    @Override
//    public int getVersion() {
//        return 0;
//    }
//
//    @Override
//    public void resetDeviceConfigurationForOpMode() {
//
//    }
//
//    @Override
//    public void close() {
//
//    }
//
//    @Override
//    public I2cController getI2cController() {
//        return null;
//    }
//
//    @Override
//    public int getPort() {
//        return 0;
//    }
//
//}