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

#include "MPU6050_6Axis_MotionApps20.h"
#include "mbed.h"
#include "config.h"
#include <stdio.h>

#include "BLE.h"
#include "UARTService.h"
#include "DFUService.h"

#define PI 3.14159265359

#define CONN_INTERVAL 25  /**< connection interval 20ms; in multiples of 0.125ms. (durationInMillis * 1000) / UNIT_0_625_MS; */
#define CONN_SUP_TIMEOUT  8000 /**< Connection supervisory timeout (6 seconds); in multiples of 0.125ms. */
#define SLAVE_LATENCY     0

#define sampleFreq  512.0f      // sample frequency in Hz
#define betaDef     0.1f        // 2 * proportional gain


MPU6050 mpu(0x69);
DigitalOut rLed(p17);
DigitalOut bLed(p19);

Timer t;
bool measure;
bool mpuInterrupt;
bool ledInterrupt;
int mpuCount = 0;

int mpuTimePassed;
int ledTimePassed;

static int timer;
static int mpuTimer;
static int ledTimer;

const int x = 0;
const int y = 1;
const int z = 2;

char DEVICENAME[6];
char *FORMATDATA;
char *FORMATVOLTAGE;

BLE   ble;
UARTService *uart;

float aRes = 2.0/32768.0;
float gRes = 250.0/32768.0;
float ax, ay, az;
float gx, gy, gz;

float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;
float q0 = 1.0f, q1 = 1.0f, q2 = 1.0f, q3 = 1.0f;


void sendString(const char *str)
{
    uart->writeString(str);
}

void sendFloat(float num)
{
    const int snprintf_buffer_size = 100;
    char snprintf_buffer[snprintf_buffer_size];
    snprintf( snprintf_buffer, snprintf_buffer_size, "%6.2f\n", num); //right
    sendString(snprintf_buffer);
}

void sendInt(const int num)
{
    char str[12];
    sprintf(str, "%d", num);
    uart->writeString(str);
}

bool Init()
{
    mpu.initialize();
    if (mpu.testConnection()) {
        //pc.printf("MPU6050 test connection passed.\n");
    } else {
        //pc.printf("MPU6050 test connection failed.\n");
        return false;
    }

    mpu.setXAccelOffset(-2937);
    mpu.setYAccelOffset(-1807);
    mpu.setZAccelOffset(1621);
    mpu.setXGyroOffsetUser(140);
    mpu.setYGyroOffsetUser(26);
    mpu.setZGyroOffsetUser(8);

    //device type

    wait(1);
    rLed = 1;
    return true;
}

void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    measure = false;
    mpuCount = 0;
    ble.startAdvertising();
}

void connectionCallback(const Gap::ConnectionCallbackParams_t *params)
{

    Gap::ConnectionParams_t gap_conn_params;
    gap_conn_params.minConnectionInterval        = Gap::MSEC_TO_GAP_DURATION_UNITS(CONN_INTERVAL);
    gap_conn_params.maxConnectionInterval        = Gap::MSEC_TO_GAP_DURATION_UNITS(CONN_INTERVAL);
    gap_conn_params.connectionSupervisionTimeout = Gap::MSEC_TO_GAP_DURATION_UNITS(CONN_SUP_TIMEOUT);
    gap_conn_params.slaveLatency                 = SLAVE_LATENCY;

    if (ble.updateConnectionParams(params->handle, &gap_conn_params) != BLE_ERROR_NONE) {
        //DEBUG("failed to update connection paramter\r\n");
    }
}

float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float quaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    
    float roll  = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    roll  *= 180.0f / PI;
    return roll;
    
}

void my_analogin_init(void)
{
    NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
    NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
                      (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                      (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                      (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos) |
                      (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
}

uint16_t my_analogin_read_u16(void)
{
    NRF_ADC->CONFIG     &= ~ADC_CONFIG_PSEL_Msk;
    NRF_ADC->CONFIG     |= ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos;
    NRF_ADC->TASKS_START = 1;
    while (((NRF_ADC->BUSY & ADC_BUSY_BUSY_Msk) >> ADC_BUSY_BUSY_Pos) == ADC_BUSY_BUSY_Busy) {};
    return (uint16_t)NRF_ADC->RESULT; // 10 bit
}

void ledOff()
{
    bLed = 1;
    rLed = 1;
}

void stopTimer()
{
    timer = 0;
    mpuTimer = 0;
    ledTimer = 0;
    mpuTimePassed = 0;
    ledTimePassed = 0;

    t.stop();
    t.reset();
}


void getMotion()
{
    mpu.setSleepEnabled(false);

    int16_t rawA[3];
    int16_t rawG[3];
    mpu.getMotion6(&rawA[x], &rawA[y], &rawA[z],&rawG[x], &rawG[y], &rawG[z]);
    ax = (float)(rawA[0])*aRes;  // get actual g value, this depends on scale being set
    ay = (float)(rawA[1])*aRes;
    az = (float)(rawA[2])*aRes;
    gx = (float)(rawG[0])*gRes;  // get actual gyro value, this depends on scale being set
    gy = (float)(rawG[1])*gRes;
    gz = (float)(rawG[2])*gRes;
    float roll = quaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f);

    if (mpuInterrupt) {
        mpuCount++;
        mpuInterrupt = false;
        
        float rollDisplay;
        rollDisplay = roll+90;
        if (rollDisplay > 90) {
            rollDisplay = 180 - rollDisplay;
        }

        const int snprintf_buffer_size = 100;
        char snprintf_buffer[snprintf_buffer_size];
        snprintf( snprintf_buffer, snprintf_buffer_size, FORMATDATA, rollDisplay, sqrt(ay*ay + az*az + ax*ax)); //right
        sendString(snprintf_buffer);

        if (mpuCount >= 6000) {
            measure = false;
            mpuCount = 0;
        }

    }

}

void sendVoltage()
{
    //voltage
    const int snprintf_buffer_size = 100;
    char snprintf_buffer[snprintf_buffer_size];

    float value = (float)my_analogin_read_u16();
    value = (value * 3.6) / 1024.0;
    //value = (value - 2.9 + 0.5)*200;
    snprintf( snprintf_buffer, snprintf_buffer_size, FORMATVOLTAGE, value);
    sendString(snprintf_buffer);
}


void ledToggle()
{
    if (ledInterrupt == true) {
        ledInterrupt = false;

        bLed = !bLed;
        rLed = !rLed;

    }
}


void onDataWritten(const GattWriteCallbackParams *params)
{
    if ((uart != NULL) && (params->handle == uart->getTXCharacteristicHandle())) {
        uint16_t bytesRead = params->len;
        mpuCount = 0;

        if (bytesRead == 3) {
            measure = !measure;
            if (measure) {
                mpu.setSleepEnabled(false);
                q0 = 1.0f;
                q1 = 1.0f;
                q2 = 1.0f;
                q3 = 1.0f;
                wait_ms(150);
                t.start();
            }
        }
        if (bytesRead == 4) {
            sendVoltage();

        }
    }
}

void bleInit()
{
    ble.init();
    ble.onDisconnection(disconnectionCallback);
    ble.onConnection(connectionCallback);
    ble.onDataWritten(onDataWritten);

    /* setup advertising */
    ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED);
    ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME,
                                     (const uint8_t *)DEVICENAME, sizeof(DEVICENAME) - 1);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                     (const uint8_t *)UARTServiceUUID_reversed, sizeof(UARTServiceUUID_reversed));

    ble.setAdvertisingInterval(1000);
    ble.startAdvertising();
    uart = new UARTService(ble);

}

void powerManage()
{
    //diable twi
    NRF_TWI1->TASKS_STOP = 1;
    NRF_TWI1->ENABLE = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;
    NRF_TWI1->INTENCLR = (TWI_INTENCLR_STOPPED_Disabled << TWI_INTENCLR_STOPPED_Pos)| \
                         (TWI_INTENCLR_RXDREADY_Disabled << TWI_INTENCLR_RXDREADY_Pos)| \
                         (TWI_INTENCLR_TXDSENT_Disabled << TWI_INTENCLR_TXDSENT_Pos)| \
                         (TWI_INTENCLR_ERROR_Disabled << TWI_INTENCLR_ERROR_Pos)| \
                         (TWI_INTENCLR_BB_Disabled << TWI_INTENCLR_BB_Pos);

    NRF_TWI0->TASKS_STOP = 1;
    NRF_TWI0->ENABLE = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;
    NRF_TWI0->INTENCLR = (TWI_INTENCLR_STOPPED_Disabled << TWI_INTENCLR_STOPPED_Pos)| \
                         (TWI_INTENCLR_RXDREADY_Disabled << TWI_INTENCLR_RXDREADY_Pos)| \
                         (TWI_INTENCLR_TXDSENT_Disabled << TWI_INTENCLR_TXDSENT_Pos)| \
                         (TWI_INTENCLR_ERROR_Disabled << TWI_INTENCLR_ERROR_Pos)| \
                         (TWI_INTENCLR_BB_Disabled << TWI_INTENCLR_BB_Pos);

    //diable spi
    NRF_SPI0->ENABLE = 0;

    //disable uart
    NRF_UART0->TASKS_STOPTX = 1;
    NRF_UART0->TASKS_STOPRX = 1;
    NRF_UART0->ENABLE = 0;

    NRF_POWER->TASKS_LOWPWR = 1;

}

void charInit()
{
    strcpy(DEVICENAME, "NRF51");
        FORMATDATA = "%6.2f_%6.2f\n";
        FORMATVOLTAGE = "v_%6.2f\n";

}

int main()
{
    MBED_ASSERT(Init() == true);
    mpu.setSleepEnabled(true);
    charInit();
    bleInit();
    //DFUService dfu(ble);
    my_analogin_init();
    wait(1);
    bLed = 1;

    int mpuInterval = 30;
    int ledInterval = 1000; //s
    int ledDuration = 50; //us

    measure = false;
    mpuInterrupt = false;
    ledInterrupt = false;

    while(true) {

        if (measure) {
            timer = t.read_ms();
            mpuTimePassed = timer - mpuTimer;
            ledTimePassed = timer - ledTimer;

            if ( mpuTimePassed >= mpuInterval) {
                mpuTimer += (mpuInterval);
                mpuInterrupt = true;
            }
            if ( ledTimePassed >= (ledInterval)) {
                ledTimer += (ledInterval);
                ledInterrupt = true;
            }
            if ( ledTimePassed >= (ledDuration) && bLed == 0 && rLed == 0 || mpuCount == 0) {
                ledInterrupt = true;
            }
            if (timer > 2000000) { //timer overflow protection (unreachable)
                stopTimer();
                t.start();
            }
            ledToggle();
            getMotion();

        } else {
            stopTimer();
            ledOff();
            mpu.setSleepEnabled(true);
            wait_ms(150);
            powerManage();
            ble.waitForEvent();
        }

    }


}

