#define VERSION "0.0.1"

#define I2C_ADDR 0x18
#define CHIP_ID 0xE26A
#define CHIP_VERSION 2

#define REG_CHIP_ID_L 0xfa
#define REG_CHIP_ID_H 0xfb
#define REG_VERSION 0xfc

// Rotary encoder
#define REG_ENC_EN 0x04
#define BIT_ENC_EN_1 0
#define BIT_ENC_MICROSTEP_1 1
#define BIT_ENC_EN_2 2
#define BIT_ENC_MICROSTEP_2 3
#define BIT_ENC_EN_3 4
#define BIT_ENC_MICROSTEP_3 5
#define BIT_ENC_EN_4 6
#define BIT_ENC_MICROSTEP_4 7

#define REG_ENC_1_CFG 0x05
#define REG_ENC_1_COUNT 0x06
#define REG_ENC_2_CFG 0x07
#define REG_ENC_2_COUNT 0x08
#define REG_ENC_3_CFG 0x09
#define REG_ENC_3_COUNT 0x0A
#define REG_ENC_4_CFG 0x0B
#define REG_ENC_4_COUNT 0x0C

// Cap touch
#define REG_CAPTOUCH_EN 0x0D
#define REG_CAPTOUCH_CFG 0x0E
#define REG_CAPTOUCH_0 0x0F  // First of 8 bytes from 15-22

// Switch counters
#define REG_SWITCH_EN_P0 0x17
#define REG_SWITCH_EN_P1 0x18
#define REG_SWITCH_P00 0x19  // First of 8 bytes from 25-40
#define REG_SWITCH_P10 0x21  // First of 8 bytes from 33-49

#define REG_USER_FLASH 0xD0
#define REG_FLASH_PAGE 0xF0
#define REG_DEBUG 0xF8

#define REG_P0 0x40       // protect_bits 2   --  Bit addressing
#define REG_SP 0x41       // Read only
#define REG_DPL 0x42      // Read only
#define REG_DPH 0x43      // Read only
#define REG_RCTRIM0 0x44  // Read only
#define REG_RCTRIM1 0x45  // Read only
#define REG_RWK 0x46
#define REG_PCON 0x47     // Read only
#define REG_TCON 0x48
#define REG_TMOD 0x49
#define REG_TL0 0x4a
#define REG_TL1 0x4b
#define REG_TH0 0x4c
#define REG_TH1 0x4d
#define REG_CKCON 0x4e
#define REG_WKCON 0x4f    // Read only
#define REG_P1 0x50       // protect_bits 3 6   --  Bit addressing
#define REG_SFRS 0x51     // TA protected   -- Read only
#define REG_CAPCON0 0x52
#define REG_CAPCON1 0x53
#define REG_CAPCON2 0x54
#define REG_CKDIV 0x55
#define REG_CKSWT 0x56    // TA protected   -- Read only
#define REG_CKEN 0x57     // TA protected   -- Read only
#define REG_SCON 0x58
#define REG_SBUF 0x59
#define REG_SBUF_1 0x5a
#define REG_EIE 0x5b      // Read only
#define REG_EIE1 0x5c     // Read only
#define REG_CHPCON 0x5f   // TA protected   -- Read only
#define REG_P2 0x60       // Bit addressing
#define REG_AUXR1 0x62
#define REG_BODCON0 0x63  // TA protected
#define REG_IAPTRG 0x64   // TA protected   -- Read only
#define REG_IAPUEN 0x65   // TA protected   -- Read only
#define REG_IAPAL 0x66    // Read only
#define REG_IAPAH 0x67    // Read only
#define REG_IE 0x68       // Read only
#define REG_SADDR 0x69
#define REG_WDCON 0x6a    // TA protected
#define REG_BODCON1 0x6b  // TA protected
#define REG_P3M1 0x6c
#define REG_P3S 0xc0      // Page 1   -- Reassigned from 0x6c to avoid collision
#define REG_P3M2 0x6d
#define REG_P3SR 0xc1     // Page 1   -- Reassigned from 0x6d to avoid collision
#define REG_IAPFD 0x6e    // Read only
#define REG_IAPCN 0x6f    // Read only
#define REG_P3 0x70       // Bit addressing
#define REG_P0M1 0x71     // protect_bits  2
#define REG_P0S 0xc2      // Page 1   -- Reassigned from 0x71 to avoid collision
#define REG_P0M2 0x72     // protect_bits  2
#define REG_P0SR 0xc3     // Page 1   -- Reassigned from 0x72 to avoid collision
#define REG_P1M1 0x73     // protect_bits  3 6
#define REG_P1S 0xc4      // Page 1   -- Reassigned from 0x73 to avoid collision
#define REG_P1M2 0x74     // protect_bits  3 6
#define REG_P1SR 0xc5     // Page 1   -- Reassigned from 0x74 to avoid collision
#define REG_P2S 0x75
#define REG_IPH 0x77      // Read only
#define REG_PWMINTC 0xc6  // Page 1   -- Read only   -- Reassigned from 0x77 to avoid collision
#define REG_IP 0x78       // Read only
#define REG_SADEN 0x79
#define REG_SADEN_1 0x7a
#define REG_SADDR_1 0x7b
#define REG_I2DAT 0x7c    // Read only
#define REG_I2STAT 0x7d   // Read only
#define REG_I2CLK 0x7e    // Read only
#define REG_I2TOC 0x7f    // Read only
#define REG_I2CON 0x80    // Read only
#define REG_I2ADDR 0x81   // Read only
#define REG_ADCRL 0x82
#define REG_ADCRH 0x83
#define REG_T3CON 0x84
#define REG_PWM4H 0xc7    // Page 1   -- Reassigned from 0x84 to avoid collision
#define REG_RL3 0x85
#define REG_PWM5H 0xc8    // Page 1   -- Reassigned from 0x85 to avoid collision
#define REG_RH3 0x86
#define REG_PIOCON1 0xc9  // Page 1   -- Reassigned from 0x86 to avoid collision
#define REG_TA 0x87       // Read only
#define REG_T2CON 0x88
#define REG_T2MOD 0x89
#define REG_RCMP2L 0x8a
#define REG_RCMP2H 0x8b
#define REG_TL2 0x8c
#define REG_PWM4L 0xca    // Page 1   -- Reassigned from 0x8c to avoid collision
#define REG_TH2 0x8d
#define REG_PWM5L 0xcb    // Page 1   -- Reassigned from 0x8d to avoid collision
#define REG_ADCMPL 0x8e
#define REG_ADCMPH 0x8f
#define REG_PSW 0x90      // Read only
#define REG_PWMPH 0x91
#define REG_PWM0H 0x92
#define REG_PWM1H 0x93
#define REG_PWM2H 0x94
#define REG_PWM3H 0x95
#define REG_PNP 0x96
#define REG_FBD 0x97
#define REG_PWMCON0 0x98
#define REG_PWMPL 0x99
#define REG_PWM0L 0x9a
#define REG_PWM1L 0x9b
#define REG_PWM2L 0x9c
#define REG_PWM3L 0x9d
#define REG_PIOCON0 0x9e
#define REG_PWMCON1 0x9f
#define REG_ACC 0xa0      // Read only
#define REG_ADCCON1 0xa1
#define REG_ADCCON2 0xa2
#define REG_ADCDLY 0xa3
#define REG_C0L 0xa4
#define REG_C0H 0xa5
#define REG_C1L 0xa6
#define REG_C1H 0xa7
#define REG_ADCCON0 0xa8
#define REG_PICON 0xa9    // Read only
#define REG_PINEN 0xaa    // Read only
#define REG_PIPEN 0xab    // Read only
#define REG_PIF 0xac      // Read only
#define REG_C2L 0xad
#define REG_C2H 0xae
#define REG_EIP 0xaf      // Read only
#define REG_B 0xb0        // Read only
#define REG_CAPCON3 0xb1
#define REG_CAPCON4 0xb2
#define REG_SPCR 0xb3
#define REG_SPCR2 0xcc    // Page 1   -- Reassigned from 0xb3 to avoid collision
#define REG_SPSR 0xb4
#define REG_SPDR 0xb5
#define REG_AINDIDS 0xb6
#define REG_EIPH 0xb7     // Read only
#define REG_SCON_1 0xb8
#define REG_PDTEN 0xb9    // TA protected
#define REG_PDTCNT 0xba   // TA protected
#define REG_PMEN 0xbb
#define REG_PMD 0xbc
#define REG_EIP1 0xbe     // Read only
#define REG_EIPH1 0xbf    // Read only

#define REG_INT 0xf9
#define MASK_INT_TRIG 0x1
#define MASK_INT_OUT 0x2
#define BIT_INT_TRIGD 0
#define BIT_INT_OUT_EN 1
#define BIT_INT_PIN_SWAP 2  // 0 = P1.3, 1 = P0.0

#define REG_INT_MASK_P0 0x00
#define REG_INT_MASK_P1 0x01
#define REG_INT_MASK_P3 0x03


#define REG_VERSION 0xfc
#define REG_ADDR 0xfd

#define REG_CTRL 0xfe     // 0 = Sleep, 1 = Reset, 2 = Read Flash, 3 = Write Flash, 4 = Addr Unlock
#define MASK_CTRL_SLEEP 0x1
#define MASK_CTRL_RESET 0x2
#define MASK_CTRL_FREAD 0x4
#define MASK_CTRL_FWRITE 0x8
#define MASK_CTRL_ADDRWR 0x10

// Special mode registers, use a bit-addressing scheme to avoid
// writing the *whole* port and smashing the i2c pins
inline bool isBitAddressedReg(uint8_t reg) {
  // 0x40, 0x50, 0x60, 0x70  -> 0b01xx0000
  return ((reg & 0b11001111) == 0b01000000);
}

// These values encode our desired pin function: IO, ADC, PWM
// alongwide the GPIO MODE for that port and pin (section 8.1)
// the 5th bit additionally encodes the default output state
#define PIN_MODE_IO      0b00000   // General IO mode, IE: not ADC or PWM
#define PIN_MODE_QB      0b00000   // Output, Quasi-Bidirectional mode
#define PIN_MODE_PP      0b00001   // Output, Push-Pull mode
#define PIN_MODE_IN      0b00010   // Input-only (high-impedance)
#define PIN_MODE_OD      0b00011   // Output, Open-Drain mode
#define PIN_MODE_PWM     0b00101   // PWM, Output, Push-Pull mode
#define PIN_MODE_ADC     0b01010   // ADC, Input-only (high-impedance)
#define PIN_MODE_PU      0b10000   // Input (with pull-up)
#define PIN_MODE_INVALID 0b10001   // Must be one greater than the highest supported pin mode.

static const char *MODE_NAMES[] = {"IO", "PWM", "ADC"};
static const char *GPIO_NAMES[] = {"QB", "PP", "IN", "OD"};
static const char *STATE_NAMES[] = {"LOW", "HIGH"};

#define IN PIN_MODE_IN
#define IN_PULL_UP PIN_MODE_PU
#define IN_PU PIN_MODE_PU
#define OUT PIN_MODE_PP
#define PWM PIN_MODE_PWM
#define ADC PIN_MODE_ADC

#define HIGH 1
#define LOW 0
