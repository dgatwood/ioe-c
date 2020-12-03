#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include "constants.h"

#ifdef __linux__
#include <linux/i2c-dev.h>

#else

#define I2C_RDWR	0x0707	/* Combined R/W transfer (one STOP only) */

struct i2c_msg {
  uint16_t addr;
  uint16_t flags;
#define I2C_M_TEN		0x0010
#define I2C_M_RD		0x0001
#define I2C_M_NOSTART		0x4000
#define I2C_M_REV_DIR_ADDR	0x2000
#define I2C_M_IGNORE_NAK	0x1000
#define I2C_M_NO_RD_ACK		0x0800
#define I2C_M_RECV_LEN		0x0400
  uint16_t len;
  uint8_t *buf;
};  

struct i2c_rdwr_ioctl_data {
    struct i2c_msg *msgs;  /* ptr to array of simple messages */
    int nmsgs;             /* number of messages to exchange */
};

#endif

#define I2C_FILENAME "/dev/i2c-1"

#undef FIXME
#undef TEN_BIT_ADDRESS  // ???

typedef struct {
    bool modeSupported[PIN_MODE_INVALID];
    int mode;
    int port;
    int pinNumber;

    // The PxM1 and PxM2 registers encode GPIO MODE
    // 0 0 = Quasi-bidirectional
    // 0 1 = Push-pull
    // 1 0 = Input-only (high-impedance)
    // 1 1 = Open-drain
    uint8_t reg_m1, reg_m2;

    // The Px input register
    uint8_t reg_p;

    // The PxS Schmitt trigger register
    uint8_t reg_ps, reg_int_mask_p;

    // ADC pin values
    int adc_channel;

    // PWM pin values
    int pwm_channel;
    uint8_t reg_iopwm;
    uint8_t reg_pwml;
    uint8_t reg_pwmh;
} pin_t;

typedef uint8_t i2c_addr_t;  // default 0x20; values from 0x20 - 0x27
typedef int i2c_dev_t;   // TYPE?
typedef int gpio_t;      // TYPE?

typedef struct {
    int fd;
    i2c_addr_t _i2c_addr;
    i2c_dev_t _i2c_dev;
    bool _debug;
    double _vref;
    double _timeout;
    int _interrupt_pin;
    gpio_t _gpio;
    int _encoder_offset[4];
    int _encoder_last[4];
    pin_t *_pins[14];
} ioexpander_t;


// Returns a 16-bit integer to distinguish errors (-1) from legitimate values.
int16_t _ioe_i2c_read8(ioexpander_t *ioe, uint8_t reg);

// Returns true on success, else false.
bool _ioe_i2c_write8(ioexpander_t *ioe, uint8_t reg, uint8_t value);

void _ioe_adc_pin_init(pin_t *pin, int port, int pinNumber, int channel);
void _ioe_pwm_pin_init(pin_t *pin, int port, int pinNumber, int channel, uint8_t reg_iopwm);
void ioe_set_mode(ioexpander_t *ioe, int pinNumber, int mode, bool schmitt_trigger /* false */, bool invert /* false */);
void _ioe_output(ioexpander_t *ioe, int pinNumber, uint32_t value);
void _ioe_set_bit(ioexpander_t *ioe, uint8_t reg, uint8_t bit);
void _ioe_clr_bits(ioexpander_t *ioe, uint8_t reg, uint8_t bits);
void _ioe_change_bit(ioexpander_t *ioe, uint8_t reg, uint8_t bit, bool state);
double _ioe_fractime(void);


pin_t *ioe_pin(int port, int pinNumber) {
    pin_t *pin = (pin_t *)malloc(sizeof(*pin));
    pin->modeSupported[PIN_MODE_IO] = true;
    pin->mode = PIN_MODE_INVALID;
    pin->port = port;
    pin->pinNumber = pinNumber;

    switch(port) {
        case 0:
            pin->reg_m1 = REG_P0M1;
            pin->reg_m2 = REG_P0M2;
            pin->reg_p = REG_P0;
            pin->reg_ps = REG_P0S;
            pin->reg_int_mask_p = REG_INT_MASK_P0;
            break;
        case 1:
            pin->reg_m1 = REG_P1M1;
            pin->reg_m2 = REG_P1M2;
            pin->reg_p = REG_P1;
            pin->reg_ps = REG_P1S;
            pin->reg_int_mask_p = REG_INT_MASK_P1;
            break;
        case 3:
            pin->reg_m1 = REG_P3M1;
            pin->reg_m2 = REG_P3M2;
            pin->reg_p = REG_P3;
            pin->reg_ps = REG_P3S;
            pin->reg_int_mask_p = REG_INT_MASK_P3;
            break;
        default:
            fprintf(stderr, "Invalid port (%d).\n", port);
            exit(1);
    }
    return pin;
}

pin_t *ioe_pwm_pin(int port, int pinNumber, int channel, uint8_t reg_iopwm) {
    pin_t *pin = ioe_pin(port, pinNumber);
    _ioe_pwm_pin_init(pin, port, pinNumber, channel, reg_iopwm);
    return pin;
}

void _ioe_pwm_pin_init(pin_t *pin, int port, int pinNumber, int channel, uint8_t reg_iopwm) {
    pin->modeSupported[PIN_MODE_PWM] = true;
    pin->pwm_channel = channel;
    pin->reg_iopwm = reg_iopwm;

    switch(channel) {
        case 0:
            pin->reg_pwml = REG_PWM0L;
            pin->reg_pwmh = REG_PWM0H;
            break;
        case 1:
            pin->reg_pwml = REG_PWM1L;
            pin->reg_pwmh = REG_PWM1H;
            break;
        case 2:
            pin->reg_pwml = REG_PWM2L;
            pin->reg_pwmh = REG_PWM2H;
            break;
        case 3:
            pin->reg_pwml = REG_PWM3L;
            pin->reg_pwmh = REG_PWM3H;
            break;
        case 4:
            pin->reg_pwml = REG_PWM4L;
            pin->reg_pwmh = REG_PWM4H;
            break;
        case 5:
            pin->reg_pwml = REG_PWM5L;
            pin->reg_pwmh = REG_PWM5H;
            break;
        default:
            fprintf(stderr, "Invalid PWM channel (%d).\n", channel);
            exit(1);
    }
}

pin_t *ioe_adc_pin(int port, int pinNumber, int channel) {
    pin_t *pin = ioe_pin(port, pinNumber);
    _ioe_adc_pin_init(pin, port, pinNumber, channel);
    return pin;
}

void _ioe_adc_pin_init(pin_t *pin, int port, int pinNumber, int channel) {
    pin->modeSupported[PIN_MODE_ADC] = true;
    pin->adc_channel = channel;
}


pin_t *ioe_adc_or_pwn_pin(int port, int pinNumber, int adcChannel, int pwmChannel, uint8_t reg_iopwm) {
    pin_t *pin = ioe_pin(port, pinNumber);
    _ioe_adc_pin_init(pin, port, pinNumber, adcChannel);
    _ioe_pwm_pin_init(pin, port, pinNumber, pwmChannel, reg_iopwm);
    return pin;
}

ioexpander_t *newIOExpander(i2c_addr_t i2c_addr, double interrupt_timeout, int interrupt_pin /* or -1 */, gpio_t gpio, bool skip_chip_id_check) {
    int fd = -1;
    if ((fd = open(I2C_FILENAME, O_RDWR)) < 0) {
        fprintf(stderr, "Could not open I2C bus.  Bailing.\n");
        return NULL;
    }
    ioexpander_t *ioe = (ioexpander_t *)malloc(sizeof(*ioe));
    bzero(ioe, sizeof(*ioe));
    ioe->fd = fd;
    ioe->_i2c_addr = i2c_addr;
#ifdef FIXME
    ioe->_i2c_dev = SMBus(1);  // TODO: Figure out what this is.
#endif
    ioe->_debug = false;
    ioe->_vref = 3.3;
    ioe->_timeout = interrupt_timeout;
    ioe->_interrupt_pin = interrupt_pin;
    ioe->_gpio = gpio;
    // ioe->_encoder_offset = {0, 0, 0, 0};
    // ioe->_encoder_last = {0, 0, 0, 0};

    if (ioe->_interrupt_pin != -1) {
#ifdef FIXME
        if (ioe->_gpio != -1) {
            import RPi.GPIO as GPIO
            ioe->_gpio = GPIO
        }
        ioe_setwarnings(ioe->_gpio, false)
        ioe_setmode(ioe->_gpio, GPIO.BCM)
        ioe_setup(ioe->_gpio, ioe->_interrupt_pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
        ioe_enable_interrupt_out(ioe)
#endif
    }

    ioe->_pins[ 0] = ioe_pwm_pin(1, 5, 5, REG_PIOCON1);            //  1
    ioe->_pins[ 1] = ioe_pwm_pin(1, 0, 2, REG_PIOCON0);            //  2
    ioe->_pins[ 2] = ioe_pwm_pin(1, 2, 0, REG_PIOCON0);            //  3
    ioe->_pins[ 3] = ioe_pwm_pin(1, 4, 1, REG_PIOCON0);            //  4
    ioe->_pins[ 4] = ioe_pwm_pin(0, 0, 3, REG_PIOCON0);            //  5
    ioe->_pins[ 5] = ioe_pwm_pin(0, 1, 4, REG_PIOCON0);            //  6
    ioe->_pins[ 6] = ioe_adc_or_pwn_pin(1, 1, 7, 1, REG_PIOCON0);  //  7
    ioe->_pins[ 7] = ioe_adc_or_pwn_pin(0, 3, 6, 5, REG_PIOCON0);  //  8
    ioe->_pins[ 8] = ioe_adc_or_pwn_pin(0, 4, 5, 3, REG_PIOCON1);  //  9
    ioe->_pins[ 9] = ioe_adc_pin(3, 0, 1);                         // 10
    ioe->_pins[10] = ioe_adc_pin(0, 6, 3);                         // 11
    ioe->_pins[11] = ioe_adc_or_pwn_pin(0, 5, 4, 2, REG_PIOCON1);  // 12
    ioe->_pins[12] = ioe_adc_pin(0, 7, 2);                         // 13
    ioe->_pins[13] = ioe_adc_pin(1, 7, 0);                         // 14

    if (!skip_chip_id_check) {
        uint16_t chip_id = (_ioe_i2c_read8(ioe, REG_CHIP_ID_H) << 8) | _ioe_i2c_read8(ioe, REG_CHIP_ID_L);
        if (chip_id != CHIP_ID) {
            fprintf(stderr, "Chip ID invalid: %04x expected: %04x.", chip_id, CHIP_ID);
            return NULL;
        }
    }
    return ioe;
}

void freeIOExpander(ioexpander_t *ioe) {
    close(ioe->fd);
    for (int i = 0; i < 13; i++) {
        free(ioe->_pins[i]);
    }
    free(ioe);
}

/** Read a single (8-bit) register from the device. */
int16_t _ioe_i2c_read8(ioexpander_t *ioe, uint8_t reg) {
    struct i2c_msg messages[2];
    struct i2c_msg *writeMessage = &messages[0];
    struct i2c_msg *readMessage = &messages[1];
    struct i2c_rdwr_ioctl_data readWriteData;
    readWriteData.msgs = messages;
    readWriteData.nmsgs = 2;

    writeMessage->addr = ioe->_i2c_addr;
    writeMessage->flags =
#ifdef TEN_BIT_ADDRESS
	I2C_M_TEN |
#endif
        0;
    writeMessage->len = 1;
    uint8_t registerBuf = reg;
    writeMessage->buf = &registerBuf;

    readMessage->addr = ioe->_i2c_addr;
    readMessage->flags =
#ifdef TEN_BIT_ADDRESS
	I2C_M_TEN |
#endif
        I2C_M_RD;
    readMessage->len = 1;
    uint8_t valueRead = 0;
    readMessage->buf = &valueRead;

    int result = 0;
    do {
        result = ioctl(ioe->fd, I2C_RDWR, &readWriteData);
    } while (result == EINTR);

    if (result < 0) {
        perror("ioctl(I2C_RDWR) in i2c_read");
        return -1;
    }

    return valueRead;
}

/** Write a single (8-bit) register to the device. */
bool _ioe_i2c_write8(ioexpander_t *ioe, uint8_t reg, uint8_t value) {
    struct i2c_msg writeMessage;
    struct i2c_rdwr_ioctl_data readWriteData;
    readWriteData.msgs = &writeMessage;
    readWriteData.nmsgs = 1;

    writeMessage.addr = ioe->_i2c_addr;
    writeMessage.flags =
#ifdef TEN_BIT_ADDRESS
	I2C_M_TEN |
#endif
        0;
    writeMessage.len = 2;
    uint8_t registerBuf[2];
    registerBuf[0] = reg;
    registerBuf[1] = value;
    writeMessage.buf = registerBuf;

    int result = 0;
    do {
        result = ioctl(ioe->fd, I2C_RDWR, &readWriteData);
    } while (result == EINTR);

    if (result < 0) {
        perror("ioctl(I2C_RDWR) in i2c_read");
        return false;
    }

    return true;
}


/** Set up a rotary encoder. */
void setup_rotary_encoder(ioexpander_t *ioe, int channel, int pinNumberA, int pinNumberB, int pinNumberC /* 0 for none */, bool count_microsteps /* false */) {
    channel -= 1;
    ioe_set_mode(ioe, pinNumberA, PIN_MODE_PU, true, false);
    ioe_set_mode(ioe, pinNumberB, PIN_MODE_PU, true, false);
    if (pinNumberC > 0) {
        ioe_set_mode(ioe, pinNumberC, PIN_MODE_OD, false, false);
        _ioe_output(ioe, pinNumberC, 0);
    }
    int reg = 0;
    switch(channel) {
        case 1: reg = REG_ENC_1_CFG; break;
        case 2: reg = REG_ENC_2_CFG; break;
        case 3: reg = REG_ENC_3_CFG; break;
        case 4: reg = REG_ENC_4_CFG; break;
        default:
            fprintf(stderr, "Invalid channel: %d\n", channel);
            exit(1);
    }
    _ioe_i2c_write8(ioe, reg, pinNumberA | (pinNumberB << 4));
    _ioe_change_bit(ioe, REG_ENC_EN, channel * 2 + 1, count_microsteps);
    _ioe_set_bit(ioe, REG_ENC_EN, channel * 2);
}

/** Read the step count from a rotary encoder. */
int read_rotary_encoder(ioexpander_t *ioe, int channel) {
    int last = ioe->_encoder_last[channel];
    uint8_t reg = 0;
    switch(channel) {
        case 1: reg = REG_ENC_1_COUNT; break;
        case 2: reg = REG_ENC_2_COUNT; break;
        case 3: reg = REG_ENC_3_COUNT; break;
        case 4: reg = REG_ENC_4_COUNT; break;
        default:
            fprintf(stderr, "Invalid channel: %d\n", channel);
            exit(1);
    }

    int value = _ioe_i2c_read8(ioe, reg);
    if (value & 0b10000000) {
        value -= 256;
    }

    if (last > 64 && value < -64) {
        ioe->_encoder_offset[channel - 1] += 256;
    }
    if (last < -64 && value > 64) {
        ioe->_encoder_offset[channel - 1] -= 256;
    }

    ioe->_encoder_last[channel - 1] = value;

    return ioe->_encoder_offset[channel - 1] + value;
}

#pragma mark - Converted down to here so far.  Partially converted the code after this.

/** Set the specified bits (using a mask) in a register. */
void _ioe_set_bits(ioexpander_t *ioe, uint8_t reg, uint8_t bits) {
    if (isBitAddressedReg(reg)) {  // What is this?
        for (int bit = 0; bit < 8; bit++) {
            if (bits & (1 << bit)) {
                _ioe_i2c_write8(ioe, reg, 0b1000 | (bit & 0b111));
            }
        }
    } else {
        uint8_t value = _ioe_i2c_read8(ioe, reg);
        usleep(1000);
        _ioe_i2c_write8(ioe, reg, value | bits);
    }
}

/** Set the specified bit (nth position from right) in a register. */
void _ioe_set_bit(ioexpander_t *ioe, uint8_t reg, uint8_t bit) {
    _ioe_set_bits(ioe, reg, (1 << bit));
}

/** Clear the specified bits (using a mask) in a register. */
void _ioe_clr_bits(ioexpander_t *ioe, uint8_t reg, uint8_t bits) {
    if (isBitAddressedReg(reg)) {
        for (int bit = 0; bit < 8; bit++) {
            if (bits & (1 << bit)) {
                _ioe_i2c_write8(ioe, reg, 0b0000 | (bit & 0b111));
            }
        }
    } else {
        uint8_t value = _ioe_i2c_read8(ioe, reg);
        usleep(1000);
        _ioe_i2c_write8(ioe, reg, value & ~bits);
    }
}

/** Clear the specified bit (nth position from right) in a register. */
void _ioe_clr_bit(ioexpander_t *ioe, uint8_t reg, uint8_t bit) {
    _ioe_clr_bits(ioe, reg, (1 << bit));
}

/** Returns the specified bit (nth position from right) from a register. */
bool _ioe_get_bit(ioexpander_t *ioe, uint8_t reg, uint8_t bit) {
    return (_ioe_i2c_read8(ioe, reg) & (1 << bit)) != 0;
}

/** Toggle one register bit on/off. */
void _ioe_change_bit(ioexpander_t *ioe, uint8_t reg, uint8_t bit, bool state) {
    if (state) {
        _ioe_set_bit(ioe, reg, bit);
    } else {
        _ioe_clr_bit(ioe, reg, bit);
    }
}

/** Enable the IOE interrupts. */
void ioe_enable_interrupt_out(ioexpander_t *ioe, bool pin_swap /* false */) {
    _ioe_set_bit(ioe, REG_INT, BIT_INT_OUT_EN);
    _ioe_change_bit(ioe, REG_INT, BIT_INT_PIN_SWAP, pin_swap);
}

/** Disable the IOE interrupt output. */
void ioe_disable_interrupt_out(ioexpander_t *ioe) {
    _ioe_clr_bit(ioe, REG_INT, BIT_INT_OUT_EN);
}

#ifdef FIXME
// I did not add interrupt support for what I'm doing, and it requires integrating an external
// library, so it wasn't worth the effort.  If somebody needs interrupt support, feel free to
// use libgpiod or similar to flesh out these functions.

bool get_gpio_pin_value(int pinNumber) {
    return true;
}

/** Get the IOE interrupt state. */
void _ioe_get_interrupt(ioexpander_t *ioe) {
    if (ioe->_interrupt_pin != -1) {
        return !get_gpio_pin_value(ioe->_interrupt_pin);
    } else {
        return _ioe_get_bit(ioe, REG_INT, BIT_INT_TRIGD);
    }
}

/** Clear the interrupt flag. */
void clear_interrupt(ioexpander_t *ioe) {
    _ioe_clr_bit(ioe, REG_INT, BIT_INT_TRIGD);
}

/**
 * Enable/disable the input interrupt on a specific pin.
 *
 * @param pin Pin from 1-14
 * @param enabled True/False for enabled/disabled
 */
void set_pin_interrupt(ioexpander_t *ioe, pin_t *pin, bool enabled) {
    if (pin < 1 || pin > (sizeof(ioe->_pins) / sizeof(ioe->_pins[0])) {
        fprintf(stderr, "Pin should be in range 1-14.");
        exit(1);
    }

    io_pin = ioe->_pins[pin - 1];

    _ioe_change_bit(ioe, io_pin->reg_int_mask_p, io_pin->pinNumber, enabled);
}

/**
 * Attach an event handler to be run on interrupt.
 * @param callback Callback function to run: callback(pin)
 */
void on_interrupt(ioexpander_t *ioe, void (*callback)()) {
    if (ioe->_interrupt_pin != -1) {
        ioe->_gpio.add_event_detect(ioe->_interrupt_pin, ioe->_gpio.FALLING, callback=callback, bouncetime=1);
    }
}

/** Wait for the IOE to finish writing non-volatile memory. */
void _ioe_wait_for_flash(ioexpander_t *ioe) {
    double t_start = _ioe_fractime();
    while (_ioe_get_interrupt(ioe) {
        if (_ioe_fractime() - t_start > ioe->_timeout) {
            fprintf(stderr, "Timed out waiting for interrupt!");
            exit(1);
        }
        usleep(1000);
    }

    t_start = _ioe_fractime();
    while (! _ioe_get_interrupt(ioe) {
        if (_ioe_fractime() - t_start > ioe->_timeout) {
            fprintf(stderr, "Timed out waiting for interrupt!");
            exit(1);
        }
        usleep(1000);
    }
}
#endif

/** Set the IOE i2c address. */
void set_i2c_addr(ioexpander_t *ioe, i2c_addr_t i2c_addr) {
    _ioe_set_bit(ioe, REG_CTRL, 4);
    _ioe_i2c_write8(ioe, REG_ADDR, i2c_addr);
    ioe->_i2c_addr = i2c_addr;
    usleep(25000);  // TODO Handle addr change IOError better
    // _ioe_wait_for_flash(ioe);
    _ioe_clr_bit(ioe, REG_CTRL, 4);
}

/** Set the ADC voltage reference. */
void set_adc_vref(ioexpander_t *ioe, double vref) {
    ioe->_vref = vref;
}

/** Get the ADC voltage reference. */
double get_adc_vref(ioexpander_t *ioe) {
    return ioe->_vref;
}

/** Get the IOE chip ID. */
uint16_t get_chip_id(ioexpander_t *ioe) {
    return (_ioe_i2c_read8(ioe, REG_CHIP_ID_H) << 8) | _ioe_i2c_read8(ioe, REG_CHIP_ID_L);
}

void _ioe_pwm_load(ioexpander_t *ioe) {
    // Load new period and duty registers into buffer
    double t_start = _ioe_fractime();
    _ioe_set_bit(ioe, REG_PWMCON0, 6);    // Set the "LOAD" bit of PWMCON0
    while (_ioe_get_bit(ioe, REG_PWMCON0, 6)) {
        usleep(1000);           // Wait for "LOAD" to complete
        if (_ioe_fractime() - t_start >= ioe->_timeout) {
            fprintf(stderr, "Timed out waiting for PWM load!");
            exit(1);
        }
    }
}

/**
 * Set PWM settings.
 * PWM is driven by the 24MHz FSYS clock by default.
 * @param divider Clock divider, one of 1, 2, 4, 8, 16, 32, 64 or 128
*/
void set_pwm_control(ioexpander_t *ioe, int divider) {
#ifdef FIXME
    try {
        pwmdiv2 = {
            1: 0b000,
            2: 0b001,
            4: 0b010,
            8: 0b011,
            16: 0b100,
            32: 0b101,
            64: 0b110,
            128: 0b111}[divider]
    except KeyError:
        fprintf(stderr, "A clock divider of %d", divider);
        exit(1);
    }

    // TODO: This currently sets GP, PWMTYP and FBINEN to 0
    // It might be desirable to make these available to the user
    // GP - Group mode enable (changes first three pairs of pAM to PWM01H and PWM01L)
    // PWMTYP - PWM type select: 0 edge-aligned, 1 center-aligned
    // FBINEN - Fault-break input enable

    _ioe_i2c_write8(ioe, REG_PWMCON1, pwmdiv2);
#endif
}

/**
 * Set the PWM period.
 *
 * The period is the point at which the PWM counter is reset to zero.
 *
 * The PWM clock runs at FSYS with a divider of 1/1.
 *
 * Also specifies the maximum value that can be set in the PWM duty cycle.
 */
void set_pwm_period(ioexpander_t *ioe, uint16_t value) {
    value &= 0xffff;
    _ioe_i2c_write8(ioe, REG_PWMPL, value & 0xff);
    _ioe_i2c_write8(ioe, REG_PWMPH, value >> 8);

    _ioe_pwm_load(ioe);
}

/** Get the current mode of a pin. */
int get_mode(ioexpander_t *ioe, int pinNumber) {
    return ioe->_pins[pinNumber - 1]->mode;
}

/**
 * Set a pin output mode.
 * @param mode   one of the supplied IN, OUT, PWM or ADC constants
 */
void ioe_set_mode(ioexpander_t *ioe, int pinNumber, int mode, bool schmitt_trigger /* false */, bool invert /* false */) {
    if (pinNumber < 1 || pinNumber > (sizeof(ioe->_pins) / sizeof(ioe->_pins[0]))) {
        fprintf(stderr, "Pin should be in range 1-14.\n");
        exit(1);
    }

    pin_t *io_pin = ioe->_pins[pinNumber - 1];
    if (io_pin->mode == mode) {
        return;
    }

    int gpio_mode = mode & 0b11;
    int io_mode = (mode >> 2) & 0b11;
    int initial_state = mode >> 4;

    if (io_mode != PIN_MODE_IO && !io_pin->modeSupported[mode]) {
        fprintf(stderr,"Pin %d does not support %s!", pinNumber, MODE_NAMES[mode]);
        exit(1);
    }

    io_pin->mode = mode;
    if (ioe->_debug) {
        printf("Setting pin %d to mode %d %s %s, state: %s", pinNumber, io_mode, MODE_NAMES[io_mode], GPIO_NAMES[gpio_mode], STATE_NAMES[initial_state]);
    }

    if (mode == PIN_MODE_PWM) {
        _ioe_set_bit(ioe, io_pin->reg_iopwm, io_pin->pwm_channel);
        _ioe_change_bit(ioe, REG_PNP, io_pin->pwm_channel, invert);
        _ioe_set_bit(ioe, REG_PWMCON0, 7);  // Set PWMRUN bit;
    } else {
        if (io_pin->modeSupported[PIN_MODE_PWM]) {
            _ioe_clr_bit(ioe, io_pin->reg_iopwm, io_pin->pwm_channel);
        }
    }

    int pm1 = _ioe_i2c_read8(ioe, io_pin->reg_m1);
    int pm2 = _ioe_i2c_read8(ioe, io_pin->reg_m2);

    // Clear the pm1 and pm2 bits
    pm1 &= 255 - (1 << io_pin->pinNumber);
    pm2 &= 255 - (1 << io_pin->pinNumber);

    // Set the new pm1 and pm2 bits according to our gpio_mode
    pm1 |= (gpio_mode >> 1) << io_pin->pinNumber;
    pm2 |= (gpio_mode & 0b1) << io_pin->pinNumber;

    _ioe_i2c_write8(ioe, io_pin->reg_m1, pm1);
    _ioe_i2c_write8(ioe, io_pin->reg_m2, pm2);

    // Set up Schmitt trigger mode on inputs
    if (mode == PIN_MODE_PU || mode == PIN_MODE_IN) {
        _ioe_change_bit(ioe, io_pin->reg_ps, io_pin->pinNumber, schmitt_trigger);
    }

    // 5th bit of mode encodes default output pin state
    _ioe_i2c_write8(ioe, io_pin->reg_p, (initial_state << 3) | io_pin->pinNumber);
}

/**
 * Read the IO pin state.
 *
 * Returns a 12-bit ADC reading if the pin is in ADC mode
 * Returns True/False if the pin is in any other input mode
 * Returns 0 if the pin is in PWM mode
 * @param adc_timeout Timeout (in seconds) for an ADC read (default 1.0)
*/
int input(ioexpander_t *ioe, int pinNumber, double adc_timeout /* 1.0 */) {
    if (pinNumber < 1 || pinNumber > (sizeof(ioe->_pins) / sizeof(ioe->_pins[0]))) {
        fprintf(stderr, "Pin should be in range 1-14.");
        exit(1);
    }

    pin_t *io_pin = ioe->_pins[pinNumber - 1];

    if (io_pin->mode == PIN_MODE_ADC) {
        if (ioe->_debug) {
            printf("Reading ADC from pin %d", pinNumber);
        }
        _ioe_clr_bits(ioe, REG_ADCCON0, 0x0f);
        _ioe_set_bit(ioe, REG_ADCCON0, io_pin->adc_channel);
        _ioe_i2c_write8(ioe, REG_AINDIDS, 0);
        _ioe_set_bit(ioe, REG_AINDIDS, io_pin->adc_channel);
        _ioe_set_bit(ioe, REG_ADCCON1, 0);

        _ioe_clr_bit(ioe, REG_ADCCON0, 7);  // ADCF - Clear the conversion complete flag
        _ioe_set_bit(ioe, REG_ADCCON0, 6);  // ADCS - Set the ADC conversion start flag

        // Wait for the ADCF conversion complete flag to be set
        double t_start = _ioe_fractime();
        while (!_ioe_get_bit(ioe, REG_ADCCON0, 7)) {
            usleep(10000);
            if (_ioe_fractime() - t_start >= adc_timeout) {
                fprintf(stderr, "Timeout waiting for ADC conversion!");
                exit(1);
            }
        }

        uint8_t hi = _ioe_i2c_read8(ioe, REG_ADCRH);
        uint8_t lo = _ioe_i2c_read8(ioe, REG_ADCRL);
        return ((hi << 4) | lo) / 4095.0 * ioe->_vref;
    } else {
        if (ioe->_debug) {
            printf("Reading IO from pin %d", pinNumber);
        }
        bool pv = _ioe_get_bit(ioe, io_pin->reg_p, io_pin->pinNumber);

        return pv ? HIGH : LOW;
    }
}

/**
 * Write an IO pin state or PWM duty cycle.
 * @param value Either True/False for OUT, or a number between 0 and PWM period for PWM.
 *
 */
void _ioe_output(ioexpander_t *ioe, int pinNumber, uint32_t value) {
    if (pinNumber < 1 || pinNumber > (sizeof(ioe->_pins) / sizeof(ioe->_pins[0]))) {
        fprintf(stderr, "Pin should be in range 1-14.");
        exit(1);
    }

    pin_t *io_pin = ioe->_pins[pinNumber - 1];

    if (io_pin->mode == PIN_MODE_PWM) {
        if (ioe->_debug) {
            printf("Outputting PWM to pin: %d", pinNumber);
        }
        _ioe_i2c_write8(ioe, io_pin->reg_pwml, value & 0xff);
        _ioe_i2c_write8(ioe, io_pin->reg_pwmh, value >> 8);
        _ioe_pwm_load(ioe);
    } else {
        if (value == LOW) {
            if (ioe->_debug) {
                printf("Outputting LOW to pin: %d", pinNumber);
            }
            _ioe_clr_bit(ioe, io_pin->reg_p, io_pin->pinNumber);
        } else if (value == HIGH) {
            if (ioe->_debug) {
                printf("Outputting HIGH to pin: %d", pinNumber);
            }
            _ioe_set_bit(ioe, io_pin->reg_p, io_pin->pinNumber);
        }
    }
}

double _ioe_fractime(void) {
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);
    uint64_t microseconds = ((unsigned long long)currentTime.tv_sec * 1000000) + currentTime.tv_usec;
    return (double)microseconds / (double)1000000.0;
}
