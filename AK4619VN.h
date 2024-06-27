#ifndef AK4619VN_H
#define AK4619VN_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Wire.h"

#define AK4619VN_ADDR 0x10
#define AK4619VN_ALT_ADDR 0x12

// ADDRESS Register page 60: 
// https://www.akm.com/content/dam/documents/products/audio/audio-codec/ak4619vn/ak4619vn-en-datasheet.pdf
#define PWRMGM 0x0      // Power Management
#define AUDFORM1 0x01   // Audio I/F Format (I2S/TDM)
#define AUDFORM2 0x02   // Audio I/F Format (Word/Slot)
#define SYSCLKSET 0x03  // System Clock Setting
#define MICGAIN1 0x04   // MIC AMP Gain L & R
#define MICGAIN2 0x05   // MIC AMP Gain L & R
#define ADC1LVOL 0x06   // ADC1 Lch Digital Volume
#define ADC1RVOL 0x07   // ADC1 Rch Digital Volume
#define ADC2LVOL 0x08   // ADC2 Lch Digital Volume
#define ADC2RVOL 0x09   // ADC2 Rch Digital Volume
#define ADCFILT 0x0A    // ADC Digital Filter Setting
#define ADCAIN 0x0B     // ADC Analog Input Setting
// #define RESERVED 0x0C  // Reserved
#define ADCMUTEHPF 0x0D // ADC Mute & HPF Control
#define DAC1LVOL 0x0E   // DAC1 Rch Digital Volume
#define DAC1RVOL 0x0F   // DAC1 Rch Digital Volume
#define DAC2LVOL 0x10   // DAC2 Lch Digital Volume
#define DAC2RVOL 0x11   // DAC2 Rch Digital Volume
#define DACDIN 0x12     // DAC Input Select Setting
#define DACDEEM 0x13    // DAC De-Emphasis Setting
#define DACMUTFLT 0x14  // DAC Mute & Filter Setting

class AK4619VN {
public:
    
    AK4619VN(TwoWire *i2c, uint8_t i2cAddress = AK4619VN_ADDR); // Constructor
    void begin(uint8_t SDA, uint8_t SCL);
    void begin(void);
    
    uint8_t setRstState(bool state);
    uint8_t pwrMgm(bool ADC2, bool ADC1, bool DAC2, bool DAC1);
    
    //###############
    typedef enum{
        AK_24BIT = 0x0,
        AK_20BIT = 0x01,
        AK_16BIT = 0x02,
        AK_32BIT = 0x03,
    } data_bit_length_t;
    
    //Bit order TDM, DCF[2:0], DSL[1:0], SLOT
    typedef enum{
        AK_I2S_STEREO = 0x0,
        AK_MSB_STEREO = 0x28,
        AK_PCM_SHRT_16B = 0x35,
        AK_PCM_SHRT_24B = 0x31,
        AK_PCM_SHRT_32B = 0x37,
        AK_PCM_LONG_16B = 0x3D,
        AK_PCM_LONG_24B = 0x39,
        AK_PCM_LONG_32B = 0x3F,
        AK_TDM128_I2S_32B = 0x57,
        AK_TDM128_MSB_32B = 0x7F,
        AK_TDM256_I2S_32B = 0x57,
    } audio_iface_format_t;
    
    typedef enum{
        AK_256FS_8_48KS = 0x0,
        AK_256FS_96KS = 0x01,
        AK_384FS_8_48KS = 0x02,
        AK_512FS_8_48KS = 0x03,
        AK_128FS_192KS = 0x04,
    } clk_fs_t;
    
    
    uint8_t audioFormatSlotLen(data_bit_length_t IDL, data_bit_length_t ODL);
    uint8_t audioFormatMode(audio_iface_format_t FORMAT);
    uint8_t sysClkSet(clk_fs_t FS, bool BICKEdg, bool SDOPH);
    uint8_t muteADCHPF(bool ATSPAD, bool AD2MUTE, bool AD1MUTE, bool AD2HPFN, bool AD1HPFN);

    //###############    
    typedef enum{
        AK_INGAIN_N6DB = 0x0,
        AK_INGAIN_N3DB = 0x01,
        AK_INGAIN_0DB = 0x02,
        AK_INGAIN_3DB = 0x03,
        AK_INGAIN_6DB = 0x04,
        AK_INGAIN_9DB = 0x05,
        AK_INGAIN_12DB = 0x06,
        AK_INGAIN_15DB = 0x07,
        AK_INGAIN_18DB = 0x08,
        AK_INGAIN_21DB = 0x09,
        AK_INGAIN_24DB = 0x0A,
        AK_INGAIN_27DB = 0x0B,
    } input_gain_t;
    
    
    uint8_t inputGain(input_gain_t ADC1L, input_gain_t ADC1R, input_gain_t ADC2L, input_gain_t ADC2R);
    uint8_t inputGainChange(bool relative, bool ADC1L, bool ADC1R, bool ADC2L, bool ADC2R, int8_t gain);
    
    //###############
    typedef enum{
        AK_DAC1B = 0x0,
        AK_DAC2B = 0x01,
        AK_DAC1L = 0x02,
        AK_DAC1R = 0x03,
        AK_DAC2L = 0x04,
        AK_DAC2R = 0x05,
        AK_OUTGAIN_0DB = 0x30,
        AK_OUT_GAIN_MUTE = 0xFF,
    } output_gain_t;
    
    uint8_t outputGain(bool relative, output_gain_t channel, int16_t gainVal);
    
    //###############
    typedef enum{
        AK_IN_DIFF = 0x0,
        AK_IN_SE1 = 0x01,
        AK_IN_SE2 = 0x02,
        AK_IN_PSDIFF = 0x02,
    } intput_conf_t;
    
    uint8_t inputConf(intput_conf_t ADC1L, intput_conf_t ADC1R, intput_conf_t ADC2L, intput_conf_t ADC2R);
    
    //###############
    typedef enum{
        AK_OUT_SDIN1 = 0x0,
        AK_OUT_SDIN2 = 0x01,
        AK_OUT_SDOUT1 = 0x02,
        AK_OUT_SDOUT2 = 0x03,
    } output_conf_t;
    
    uint8_t outputConf(output_conf_t DAC2, output_conf_t DAC1);


    //##################
    const char controlParams[20][11] = 
    {
    "PWRMGM",
    "AUDFORM1",
    "AUDFORM2",
    "SYSCLKSET",
    "MICGAIN1",
    "MICGAIN2",
    "ADC1LVOL",
    "ADC1RVOL",
    "ADC2LVOL",
    "ADC2RVOL",
    "ADCFILT",
    "ADCAIN",
    "ADCMUTEHPF",
    "DAC1LVOL",
    "DAC1RVOL",
    "DAC2LVOL",
    "DAC2RVOL",
    "DACDIN",
    "DACDEEM",
    "DACMUTFLT"
    };

    uint8_t printRegs(uint8_t startReg, uint8_t len);
    
    
private:
    TwoWire *_wirePort;
    uint8_t _i2cAddress;
    
    #if ESP_ARDUINO_VERSION_MAJOR == 1
    //For Arduino-ESP32 v1+
    uint8_t writeReg(uint8_t deviceReg, uint8_t regVal);
    uint8_t readReg(uint8_t deviceReg, uint8_t * regVal);
    uint8_t readRegMulti(uint8_t startReg, uint8_t len, uint8_t * vals);
    #endif
    
    
    #if ESP_ARDUINO_VERSION_MAJOR == 2  
    //For Arduino-ESP32 v2+
    uint8_t writeReg(uint8_t deviceReg, uint8_t regVal);
    uint8_t readReg(uint8_t deviceReg, uint8_t * regVal);
    uint8_t readRegMulti(uint8_t startReg, uint8_t len, uint8_t * vals);
    #endif
    
    #if not defined ESP_ARDUINO_VERSION_MAJOR //Default for non-ESP32 platforms
    // For Teensy or non-ESP32 platforms
    uint8_t writeReg(uint8_t deviceReg, uint8_t regVal);
    uint8_t readReg(uint8_t deviceReg, uint8_t * regVal);
    uint8_t readRegMulti(uint8_t startReg, uint8_t len, uint8_t * vals);
    #endif

    uint8_t modifyGainRange(int16_t inVal, uint8_t regVal);
    uint8_t checkGainRange(int16_t inVal);
    TwoWire *m_i2c;
    uint8_t m_addr;    
};

#endif // AK4619VN_H
