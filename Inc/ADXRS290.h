#ifndef ADXRS290_H_
#define ADXRS290_H_

/**
  ******************************************************************************
  * @file           ADXRS290.h
  * @brief          ADXRS290 driver definitions
  ******************************************************************************
*/

#define ADXRS290_COUNT				4
/*Schematic selector count*/
#define ADXRS290_A_COUNT			5

/* ADXRS290 registers addresses */
#define ADXRS290_ANALOG_ID        0x00
#define ADXRS290_ANALOG_ID_RETURN 0xAD

#define ADXRS290_MEMS_ID          0x01
#define ADXRS290_MEMS_ID_RETURN   0x1D

#define ADXRS290_DEV_ID           0x02
#define ADXRS290_DEV_ID_RETURN    0x92

#define ADXRS290_REV_NUM          0x03
#define ADXRS290_REV_NUM_RETURN   0x1D

#define ADXRS290_SERIALNUM_START  0x04
#define ADXRS290_SERIALNUM_END    0x07


// 16 2-complement's bits
#define ADXRS290_GYR_X_L          0x08
#define ADXRS290_GYR_X_H          0x09

// 16 2-complement's bits
#define ADXRS290_GYR_Y_L          0x0A
#define ADXRS290_GYR_Y_H          0x0B

// 12 2-complement's bits
#define ADXRS290_TEMP_L           0x0C  // 7..0 bits
#define ADXRS290_TEMP_H           0x0D  // 11..8 bits
#define ADXRS290_TEMP_SCALE_FACT  10.0  // 10 LSB /centig deg. 0 mean s o deg.

// The LSB controls Temperature sensor. 1 enable, 0 disable
#define ADXRS290_POW_CTRL_REG     0x10
#define ADXRS290_POW_CTRL_TEMP_EN_MASK  0x01

// 1: measurement mode, 0 chip in standby mode
#define ADXRS290_POW_CTRL_STDBY_MASK    0x03

// 1: measurement mode, 0 chip in standby mode
#define ADXRS290_BANDPASS_FILTER  0x11
#define ADXRS290_BPF_LPF_MASK     0x07
#define ADXRS290_BPF_HPF_MASK     0xF0
#define ADXRS290_BPF_HPF_OFFSET   0x4

#define ADXRS290_DATA_READY_REG   0x12

/* Set this bit to get triggered on data ready via interrupt
 * Set bit to 01 to gen rata ready interrupt
 * at the sync/asel pin when data becomes avail
 * Sync bits meaning:
 * X0 = Read for analog enable
 * 01 Data ready, high until read
 */
#define ADXRS290_DATA_READY_INT_MASK    0x03



/*Sensors Sensitivity */

/*
 * Low-Pass Filter Pole Locations
 *  The data is the Frequency in Hz
 */
#define ADXRS_LPF_480_HZ          0x00   //  480_HZ is Default
#define ADXRS_LPF_320_HZ          0x01   //  320_HZ
#define ADXRS_LPF_160_HZ          0x02   //  160_HZ
#define ADXRS_LPF_80_HZ           0x03   //  80_HZ
#define ADXRS_LPF_56_6_HZ         0x04   //  56.6_HZ
#define ADXRS_LPF_40_HZ           0x05   //  40 HZ
#define ADXRS_LPF_28_3_HZ         0x06   //  28.3_HZ
#define ADXRS_LPF_20_HZ           0x07   //  20_HZ

/*
 * High-Pass Filter Pole Locations
 *  The data is the Frequency in Hz
 */
#define ADXRS_HPF_ALL_HZ          0x00   // All Pass Default
#define ADXRS_HPF_0_011_HZ        0x01   //  0.011_Hz
#define ADXRS_HPF_0_022_HZ        0x02   //  0.022_Hz
#define ADXRS_HPF_0_044_HZ        0x03   //  0.044_Hz
#define ADXRS_HPF_0_087_HZ        0x04   //  0.087_Hz
#define ADXRS_HPF_0_175_HZ        0x05   //  0.187_Hz
#define ADXRS_HPF_0_350_HZ        0x06   //  0.350_Hz
#define ADXRS_HPF_0_700_HZ        0x07   //  0.700_Hz
#define ADXRS_HPF_1_400_HZ        0x08   //  1.400_Hz
#define ADXRS_HPF_2_800_HZ        0x09   //  2.800_Hz
#define ADXRS_HPF_11_30_HZ 0x0A // 11.300_Hz

/* Device data */
typedef struct
{
	uint32_t X;
	uint32_t Y;
	uint32_t T;
} ADXRS290Device;

void ADXRS290_Init(void);
void ADXRS290_Data_Scan(ADXRS290Device *self, uint8_t count);

#endif /* ADXRS290_H_ */
