
#define AVG_FACTOR  1      // SCH1 sample averaging


#define FILTER_RATE         235.0f       // Hz, LPF1 Nominal Cut-off Frequency (-3dB).
#define FILTER_ACC12        235.0f
#define FILTER_ACC3         235.0f
#define SENSITIVITY_RATE1   1600.0f     // LSB / dps, DYN1 Nominal Sensitivity for 20 bit data.
#define SENSITIVITY_RATE2   1600.0f
#define SENSITIVITY_ACC1    3200.0f     // LSB / m/s2, DYN1 Nominal Sensitivity for 20 bit data.
#define SENSITIVITY_ACC2    3200.0f
#define SENSITIVITY_ACC3    3200.0f     // LSB / m/s2, DYN1 Nominal Sensitivity for 20 bit data.
#define DECIMATION_RATE     16          // Output rate = ~23.77 kHz / decimation: 32 -> 743 Hz, 16 -> 1486 Hz (see README.md)
#define DECIMATION_ACC      16
