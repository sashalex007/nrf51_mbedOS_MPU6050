/* Demo code for MPU6050 DMP
 * I thank Ian Hua.
 * Copyright (c) 2015 Match
 *
 * THE PROGRAM IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE PROGRAM OR THE USE OR OTHER DEALINGS IN
 * THE PROGRAM.
 */
 
 
// Define Necessary.
#define OUTPUT_QUATERNION
//#define OUTPUT_EULER
//#define OUTPUT_ROLL_PITCH_YAW
//#define OUTPUT_FOR_TEAPOT
//#define OUTPUT_TEMPERATURE
 
 
#include "MPU6050_6Axis_MotionApps20.h"
#include "mbed.h"
#include "config.h"
#include <stdio.h>

#include "BLE.h"
 
#define DEG_TO_RAD(x) ( x * 0.01745329 )
#define RAD_TO_DEG(x) ( x * 57.29578 )
 
 
RawSerial pc(USBTX, USBRX);
MPU6050 mpu(I2C_SDA0, I2C_SCL0);     // sda, scl pin
 
const int FIFO_BUFFER_SIZE = 128;
uint8_t fifoBuffer[FIFO_BUFFER_SIZE];
uint16_t fifoCount;
uint16_t packetSize;
bool dmpReady;
uint8_t mpuIntStatus;
const int snprintf_buffer_size = 100;
char snprintf_buffer[snprintf_buffer_size];
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
 
struct Offset {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
}offset = {150, -350, 1000, -110, 5, 0};    // Measured values
 
struct MPU6050_DmpData {
    Quaternion q;
    VectorFloat gravity;    // g
    float roll, pitch, yaw;     // rad
}dmpData;
 
#define CONN_INTERVAL 25  /**< connection interval 20ms; in multiples of 0.125ms. (durationInMillis * 1000) / UNIT_0_625_MS; */
#define CONN_SUP_TIMEOUT  8000 /**< Connection supervisory timeout (6 seconds); in multiples of 0.125ms. */
#define SLAVE_LATENCY     0
//#define TICKER_INTERVAL   2.0f

BLE   ble;

static const char DEVICENAME[] = "TY51822r3";
static volatile bool  triggerSensorPolling = false;

static const char QUOTERNION_DESC[] = "quoternion";


//const uint8_t MPU6050_adv_service_uuid[] = {
//    0x9F,0xDF,0x32,0x83,
//    0x90,0x49,
//    0xCF,0x8D,
//    0x5C,0x4D,    
//    0x98,0xE7,0xE2,0x00,0x27,0x31
//};

static const uint8_t MPU6050_service_uuid[] = {
    0x45,0x35,0x56,0x80,0x0F,0xD8,0x5F,0xB5,0x51,0x48,0x30,0x27,0x06,0x9B,0x3F,0xD9
};

static const uint8_t MPU6050_dmp_Characteristic_uuid[] = {
    0x45,0x35,0x56,0x81,0x0F,0xD8,0x5F,0xB5,0x51,0x48,0x30,0x27,0x06,0x9B,0x3F,0xD9
};


uint8_t quoternionPayload[sizeof(float)*4] = {0,};

GattAttribute       quoternionAttr (BLE_UUID_DESCRIPTOR_CHAR_USER_DESC, (uint8_t *)QUOTERNION_DESC, sizeof(QUOTERNION_DESC), sizeof(QUOTERNION_DESC));
GattAttribute       *quoternionAttrs[] = { &quoternionAttr, };
GattCharacteristic  quoternionChar (MPU6050_dmp_Characteristic_uuid,
                                        quoternionPayload, (sizeof(float) * 4), (sizeof(float) * 4),
                                        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ, quoternionAttrs, 1);

GattCharacteristic *ControllerChars[] = { &quoternionChar, };
GattService         MPU6050DMPService(MPU6050_service_uuid, ControllerChars, sizeof(ControllerChars) / sizeof(GattCharacteristic *));


bool Init() {
    pc.baud(PC_BAUDRATE);
        
    mpu.initialize();
    if (mpu.testConnection()) {
        pc.printf("MPU6050 test connection passed.\n");
    } else {
        pc.printf("MPU6050 test connection failed.\n");
        return false;
    }
    if (mpu.dmpInitialize() == 0) {
        pc.printf("succeed in MPU6050 DMP Initializing.\n");
    } else {
        pc.printf("failed in MPU6050 DMP Initializing.\n");
        return false;
    }
    mpu.setXAccelOffset(offset.ax);
    mpu.setYAccelOffset(offset.ay);
    mpu.setZAccelOffset(offset.az);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS);
    mpu.setXGyroOffsetUser(offset.gx);
    mpu.setYGyroOffsetUser(offset.gy);
    mpu.setZGyroOffsetUser(offset.gz);
    mpu.setDMPEnabled(true);    // Enable DMP
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;    // Enable interrupt.
    
    pc.printf("Init finish!\n");
    
    return true;
}
 
void bleUpdatedValue(MPU6050_DmpData dmpData)
{
    memcpy(quoternionPayload+sizeof(float)*0, &dmpData.q.w, sizeof(float));
    memcpy(quoternionPayload+sizeof(float)*1, &dmpData.q.x, sizeof(float));
    memcpy(quoternionPayload+sizeof(float)*2, &dmpData.q.y, sizeof(float));
    memcpy(quoternionPayload+sizeof(float)*3, &dmpData.q.y, sizeof(float));
        
    ble.updateCharacteristicValue(quoternionChar.getValueAttribute().getHandle(), quoternionPayload, sizeof(quoternionPayload));    //Mod
    mpu.resetFIFO();
}
 
void dmpDataUpdate() {
    // Check that this interrupt has enabled.
    if (dmpReady == false) return;
    
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    // Check that this interrupt is a FIFO buffer overflow interrupt.
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        pc.printf("FIFO overflow!\n");
        return;
        
    // Check that this interrupt is a Data Ready interrupt.
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
    #ifdef OUTPUT_QUATERNION
        mpu.dmpGetQuaternion(&dmpData.q, fifoBuffer);
        if ( snprintf( snprintf_buffer, snprintf_buffer_size, "Quaternion : w=%f, x=%f, y=%f, z=%f\n", dmpData.q.w, dmpData.q.x, dmpData.q.y, dmpData.q.z ) < 0 ) return;
        pc.puts(snprintf_buffer);
        bleUpdatedValue(dmpData);
    #endif
        
    #ifdef OUTPUT_EULER
        float euler[3];
        mpu.dmpGetQuaternion(&dmpData.q, fifoBuffer);
        mpu.dmpGetEuler(euler, &dmpData.q);
        if ( snprintf( snprintf_buffer, snprintf_buffer_size, "Euler : psi=%fdeg, theta=%fdeg, phi=%fdeg\n", RAD_TO_DEG(euler[0]), RAD_TO_DEG(euler[1]), RAD_TO_DEG(euler[2]) ) < 0 ) return;
        pc.puts(snprintf_buffer);
    #endif
        
    #ifdef OUTPUT_ROLL_PITCH_YAW
        mpu.dmpGetQuaternion(&dmpData.q, fifoBuffer);
        mpu.dmpGetGravity(&dmpData.gravity, &dmpData.q);
        float rollPitchYaw[3];
        mpu.dmpGetYawPitchRoll(rollPitchYaw, &dmpData.q, &dmpData.gravity);
        dmpData.roll = rollPitchYaw[2];
        dmpData.pitch = rollPitchYaw[1];
        dmpData.yaw = rollPitchYaw[0];
        
        if ( snprintf( snprintf_buffer, snprintf_buffer_size, "Roll:%6.2fdeg, Pitch:%6.2fdeg, Yaw:%6.2fdeg\n", RAD_TO_DEG(dmpData.roll), RAD_TO_DEG(dmpData.pitch), RAD_TO_DEG(dmpData.yaw) ) < 0 ) return;
        pc.printf(snprintf_buffer);
    #endif
        
    #ifdef OUTPUT_FOR_TEAPOT
        teapotPacket[2] = fifoBuffer[0];
        teapotPacket[3] = fifoBuffer[1];
        teapotPacket[4] = fifoBuffer[4];        
        teapotPacket[5] = fifoBuffer[5];
        teapotPacket[6] = fifoBuffer[8];
        teapotPacket[7] = fifoBuffer[9];
        teapotPacket[8] = fifoBuffer[12];
        teapotPacket[9] = fifoBuffer[13];
        for (uint8_t i = 0; i < 14; i++) {
            pc.putc(teapotPacket[i]);
        }
    #endif
        
    #ifdef OUTPUT_TEMPERATURE
        float temp = mpu.getTemperature() / 340.0 + 36.53;
        if ( snprintf( snprintf_buffer, snprintf_buffer_size, "Temp:%4.1fdeg\n", temp ) < 0 ) return;
        pc.puts(snprintf_buffer);
    #endif
        
        pc.printf("\n");
    }
}

void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)    // Mod
{
    
    //DEBUG("Disconnected handle %u, reason %u\n", handle, reason);
    //DEBUG("Restarting the advertising process\n\r");

    ble.startAdvertising();
}

void connectionCallback(const Gap::ConnectionCallbackParams_t *params)
{

    //DEBUG("connected. Got handle %u\r\n", handle);

    /*******************************************************************************/
    /*  CentralがMacOS X の時 connection intervalを設定する場合は                      */
    /*  nRF51822 -> projectconfig.h -> GAP ->                                      */
    /*  CFG_GAP_CONNECTION_MIN_INTERVAL_MS / CFG_GAP_CONNECTION_MAX_INTERVAL_MSを  */
    /*  直接編集すること                                                             */
    /******************************************************************************/
        //Gap::Handle_t handle;
        Gap::ConnectionParams_t gap_conn_params;
        gap_conn_params.minConnectionInterval        = Gap::MSEC_TO_GAP_DURATION_UNITS(CONN_INTERVAL);
        gap_conn_params.maxConnectionInterval        = Gap::MSEC_TO_GAP_DURATION_UNITS(CONN_INTERVAL);
        gap_conn_params.connectionSupervisionTimeout = Gap::MSEC_TO_GAP_DURATION_UNITS(CONN_SUP_TIMEOUT);
        gap_conn_params.slaveLatency                 = SLAVE_LATENCY;
        
    if (ble.updateConnectionParams(params->handle, &gap_conn_params) != BLE_ERROR_NONE) {
        //DEBUG("failed to update connection paramter\r\n");
    }
}

void timeoutCallback(const Gap::TimeoutSource_t source)
{
    //DEBUG("TimeOut\n\r");
    //DEBUG("Restarting the advertising process\n\r");    

    ble.startAdvertising();
}

void bleInit(){
    ble.init();
    ble.onDisconnection(disconnectionCallback);
    ble.onConnection(connectionCallback);
    //ble.onDataWritten(writtenCallback);
    ble.onTimeout(timeoutCallback);

    /* setup device name */
    ble.setDeviceName((const uint8_t *)DEVICENAME);
    
    /* setup advertising */
    ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED  | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (const uint8_t *)DEVICENAME, sizeof(DEVICENAME));
    ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                    (const uint8_t *)MPU6050_service_uuid, sizeof(MPU6050_service_uuid));
                                    //(const uint8_t *)MPU6050_adv_service_uuid, sizeof(MPU6050_adv_service_uuid));

    ble.setAdvertisingInterval(160); /* 100ms; in multiples of 0.625ms. */
    ble.startAdvertising();

    ble.addService(MPU6050DMPService);
}

void periodicCallback(void)
{
    //oneSecondLed = !oneSecondLed; /* Do blinky on LED1 while we're waiting for BLE events */

    /* Note that the periodicCallback() executes in interrupt context, so it is safer to do
     * heavy-weight sensor polling from the main thread. */
    triggerSensorPolling = true;
}


int main() {
    MBED_ASSERT(Init() == true);
    
    Ticker ticker;
    ticker.attach(periodicCallback, 0.1f);
    
    bleInit();
    
    while(true) {
        if (triggerSensorPolling && ble.getGapState().connected) {
            triggerSensorPolling = false;
            dmpDataUpdate();
        } else {
            ble.waitForEvent();
        }

    }
}
 
