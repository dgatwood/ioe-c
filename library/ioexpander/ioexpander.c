#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <strings.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "constants.h"

#ifdef __linux__
#include <linux/i2c.h>
#include <sys/ioctl.h>
#else

#define I2C_RDWR 42 // Bogus, but just to get the code to compile.

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

typedef int i2c_addr_t;  // TYPE?
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


pin_t *ioe_pin(int port, int pinNumber) {
    pin_t *pin = malloc(sizeof(*pin));
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

ioexpander_t *newIOExpander(i2c_addr_t i2c_addr, double interrupt_timeout, int interrupt_pin, gpio_t gpio, bool skip_chip_id_check) {
    int fd = -1;
    if ((fd = open(I2C_FILENAME, O_RDWR)) < 0) {
        fprintf(stderr, "Could not open I2C bus.  Bailing.\n");
        exit(1);
    }
    ioexpander_t *ioe = malloc(sizeof(*ioe));
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

    if (ioe->_interrupt_pin != -1)
#ifdef FIXME
        if ioe->_gpio is None:
            import RPi.GPIO as GPIO
            ioe->_gpio = GPIO
        ioe_setwarnings(ioe->_gpio, false)
        ioe_setmode(ioe->_gpio, GPIO.BCM)
        ioe_setup(ioe->_gpio, ioe->_interrupt_pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
        ioe_enable_interrupt_out(ioe)
#endif

    ioe->_pins[ 0] = ioe_pwm_pin(1, 5, 5, REG_PIOCON1);
    ioe->_pins[ 1] = ioe_pwm_pin(1, 0, 2, REG_PIOCON0);
    ioe->_pins[ 2] = ioe_pwm_pin(1, 2, 0, REG_PIOCON0);
    ioe->_pins[ 3] = ioe_pwm_pin(1, 4, 1, REG_PIOCON0);
    ioe->_pins[ 4] = ioe_pwm_pin(0, 0, 3, REG_PIOCON0);
    ioe->_pins[ 5] = ioe_pwm_pin(0, 1, 4, REG_PIOCON0);
    ioe->_pins[ 6] = ioe_adc_or_pwn_pin(1, 1, 7, 1, REG_PIOCON0);
    ioe->_pins[ 7] = ioe_adc_or_pwn_pin(0, 3, 6, 5, REG_PIOCON0);
    ioe->_pins[ 8] = ioe_adc_or_pwn_pin(0, 4, 5, 3, REG_PIOCON1);
    ioe->_pins[ 9] = ioe_adc_pin(3, 0, 1);
    ioe->_pins[10] = ioe_adc_pin(0, 6, 3);
    ioe->_pins[11] = ioe_adc_or_pwn_pin(0, 5, 4, 2, REG_PIOCON1);
    ioe->_pins[12] = ioe_adc_pin(0, 7, 2);
    ioe->_pins[13] = ioe_adc_pin(1, 7, 0);

    if (!skip_chip_id_check) {
        uint16_t chip_id = (_ioe_i2c_read8(ioe, REG_CHIP_ID_H) << 8) | _ioe_i2c_read8(ioe, REG_CHIP_ID_L);
        if (chip_id != CHIP_ID) {
            fprintf(stderr, "Chip ID invalid: %04x expected: %04x.", chip_id, CHIP_ID);
            exit(1);
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
    } while (result != EINTR);

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
    } while (result != EINTR);

    if (result < 0) {
        perror("ioctl(I2C_RDWR) in i2c_read");
        return false;
    }

    return true;
}

#pragma mark - Converted down to here so far.

    def setup_rotary_encoder(self, channel, pin_a, pin_b, pin_c=None, count_microsteps=False):
        /** Set up a rotary encoder. */
        channel -= 1
        ioe->set_mode(pin_a, PIN_MODE_PU, schmitt_trigger=True)
        ioe->set_mode(pin_b, PIN_MODE_PU, schmitt_trigger=True)
        if pin_c is not None:
            ioe->set_mode(pin_c, PIN_MODE_OD)
            ioe->output(pin_c, 0)

        ioe->_ioe_i2c_write8([REG_ENC_1_CFG, REG_ENC_2_CFG, REG_ENC_3_CFG, REG_ENC_4_CFG][channel], pin_a | (pin_b << 4))
        ioe->change_bit(REG_ENC_EN, channel * 2 + 1, count_microsteps)
        ioe->set_bit(REG_ENC_EN, channel * 2)

    def read_rotary_encoder(self, channel):
        /** Read the step count from a rotary encoder. */
        channel -= 1
        last = ioe->_encoder_last[channel]
        reg = [REG_ENC_1_COUNT, REG_ENC_2_COUNT, REG_ENC_3_COUNT, REG_ENC_4_COUNT][channel]
        value = _ioe_i2c_read8(ioe, reg)

        if value & 0b10000000:
            value -= 256

        if last > 64 and value < -64:
            ioe->_encoder_offset[channel] += 256
        if last < -64 and value > 64:
            ioe->_encoder_offset[channel] -= 256

        ioe->_encoder_last[channel] = value

        return ioe->_encoder_offset[channel] + value

    def set_bits(self, reg, bits):
        /** Set the specified bits (using a mask) in a register. */
        if reg in BIT_ADDRESSED_REGS:
            for bit in range(8):
                if bits & (1 << bit):
                    ioe->_ioe_i2c_write8(reg, 0b1000 | (bit & 0b111))
        else:
            value = _ioe_i2c_read8(ioe, reg)
            time.sleep(0.001)
            ioe->_ioe_i2c_write8(reg, value | bits)

    def set_bit(self, reg, bit):
        /** Set the specified bit (nth position from right) in a register. */
        ioe->set_bits(reg, (1 << bit))

    def clr_bits(self, reg, bits):
        /** Clear the specified bits (using a mask) in a register. */
        if reg in BIT_ADDRESSED_REGS:
            for bit in range(8):
                if bits & (1 << bit):
                    ioe->_ioe_i2c_write8(reg, 0b0000 | (bit & 0b111))
        else:
            value = _ioe_i2c_read8(ioe, reg)
            time.sleep(0.001)
            ioe->_ioe_i2c_write8(reg, value & ~bits)

    def clr_bit(self, reg, bit):
        /** Clear the specified bit (nth position from right) in a register. */
        ioe->clr_bits(reg, (1 << bit))

    def get_bit(self, reg, bit):
        /** Returns the specified bit (nth position from right) from a register. */
        return _ioe_i2c_read8(ioe, reg) & (1 << bit)

    def change_bit(self, reg, bit, state):
        /** Toggle one register bit on/off. */
        if state:
            ioe->set_bit(reg, bit)
        else:
            ioe->clr_bit(reg, bit)

    def ioe_enable_interrupt_out(self, pin_swap=False):
        /** Enable the IOE interrupts. */
        ioe->set_bit(REG_INT, BIT_INT_OUT_EN)
        ioe->change_bit(REG_INT, BIT_INT_PIN_SWAP, pin_swap)

    def ioe_disable_interrupt_out(self):
        /** Disable the IOE interrupt output. */
        ioe->clr_bit(REG_INT, BIT_INT_OUT_EN)

    def get_interrupt(self):
        /** Get the IOE interrupt state. */
        if ioe->_interrupt_pin is not None:
            return ioe->_gpio.input(ioe->_interrupt_pin) == 0
        else:
            return ioe->get_bit(REG_INT, BIT_INT_TRIGD)

    def clear_interrupt(self):
        /** Clear the interrupt flag. */
        ioe->clr_bit(REG_INT, BIT_INT_TRIGD)

    def set_pin_interrupt(self, pin, enabled):
        /** Enable/disable the input interrupt on a specific pin.

        :param pin: Pin from 1-14
        :param enabled: True/False for enabled/disabled

         */
        if pin < 1 or pin > sizeof(ioe->_pins / sizeof(ioe->_pins[0])):
            raise ValueError("Pin should be in range 1-14.")

        io_pin = ioe->_pins[pin - 1]

        ioe->change_bit(io_pin.reg_int_mask_p, io_pin.pin, enabled)

    def on_interrupt(self, callback):
        /** Attach an event handler to be run on interrupt.

        :param callback: Callback function to run: callback(pin)

         */
        if ioe->_interrupt_pin is not None:
            ioe->_gpio.add_event_detect(ioe->_interrupt_pin, ioe->_gpio.FALLING, callback=callback, bouncetime=1)

    def _wait_for_flash(self):
        /** Wait for the IOE to finish writing non-volatile memory. */
        t_start = time.time()
        while ioe->get_interrupt():
            if time.time() - t_start > ioe->_timeout:
                raise RuntimeError("Timed out waiting for interrupt!")
            time.sleep(0.001)

        t_start = time.time()
        while not ioe->get_interrupt():
            if time.time() - t_start > ioe->_timeout:
                raise RuntimeError("Timed out waiting for interrupt!")
            time.sleep(0.001)

    def set_i2c_addr(self, i2c_addr):
        /** Set the IOE i2c address. */
        ioe->set_bit(REG_CTRL, 4)
        ioe->_ioe_i2c_write8(REG_ADDR, i2c_addr)
        ioe->_i2c_addr = i2c_addr
        time.sleep(0.25)  // TODO Handle addr change IOError better
        // ioe->_wait_for_flash()
        ioe->clr_bit(REG_CTRL, 4)

    def set_adc_vref(self, vref):
        /** Set the ADC voltage reference. */
        ioe->_vref = vref

    def get_adc_vref(self):
        /** Get the ADC voltage reference. */
        return ioe->_vref

    def get_chip_id(self):
        /** Get the IOE chip ID. */
        return (_ioe_i2c_read8(ioe, REG_CHIP_ID_H) << 8) | _ioe_i2c_read8(ioe, REG_CHIP_ID_L)

    def _pwm_load(self):
        // Load new period and duty registers into buffer
        t_start = time.time()
        ioe->set_bit(REG_PWMCON0, 6)    // Set the "LOAD" bit of PWMCON0
        while ioe->get_bit(REG_PWMCON0, 6):
            time.sleep(0.001)           // Wait for "LOAD" to complete
            if time.time() - t_start >= ioe->_timeout:
                raise RuntimeError("Timed out waiting for PWM load!")

    def set_pwm_control(self, divider):
        /** Set PWM settings.

        PWM is driven by the 24MHz FSYS clock by default.

        :param divider: Clock divider, one of 1, 2, 4, 8, 16, 32, 64 or 128

         */
        try:
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
            raise ValueError("A clock divider of {}".format(divider))

        // TODO: This currently sets GP, PWMTYP and FBINEN to 0
        // It might be desirable to make these available to the user
        // GP - Group mode enable (changes first three pairs of pAM to PWM01H and PWM01L)
        // PWMTYP - PWM type select: 0 edge-aligned, 1 center-aligned
        // FBINEN - Fault-break input enable

        ioe->_ioe_i2c_write8(REG_PWMCON1, pwmdiv2)

    def set_pwm_period(self, value):
        /** Set the PWM period.

        The period is the point at which the PWM counter is reset to zero.

        The PWM clock runs at FSYS with a divider of 1/1.

        Also specifies the maximum value that can be set in the PWM duty cycle.

         */
        value &= 0xffff
        ioe->_ioe_i2c_write8(REG_PWMPL, value & 0xff)
        ioe->_ioe_i2c_write8(REG_PWMPH, value >> 8)

        ioe->_pwm_load()

    def get_mode(self, pin):
        /** Get the current mode of a pin. */
        return ioe->_pins[pin - 1].mode

    def set_mode(self, pin, mode, schmitt_trigger=False, invert=False):
        /** Set a pin output mode.

        :param mode: one of the supplied IN, OUT, PWM or ADC constants

         */
        if pin < 1 or pin > sizeof(ioe->_pins / sizeof(ioe->_pins[0]):
            raise ValueError("Pin should be in range 1-14.")

        io_pin = ioe->_pins[pin - 1]
        if io_pin.mode == mode:
            return

        gpio_mode = mode & 0b11
        io_mode = (mode >> 2) & 0b11
        initial_state = mode >> 4

        if io_mode != PIN_MODE_IO and mode not in io_pin.type:
            raise ValueError("Pin {} does not support {}!".format(pin, MODE_NAMES[io_mode]))

        io_pin.mode = mode
        if ioe->_debug:
            print("Setting pin {pin} to mode {mode} {name}, state: {state}".format(pin=pin, mode=MODE_NAMES[io_mode], name=GPIO_NAMES[gpio_mode], state=STATE_NAMES[initial_state]))

        if mode == PIN_MODE_PWM:
            ioe->set_bit(io_pin.reg_iopwm, io_pin.pwm_channel)
            ioe->change_bit(REG_PNP, io_pin.pwm_channel, invert)
            ioe->set_bit(REG_PWMCON0, 7)  // Set PWMRUN bit

        else:
            if PIN_MODE_PWM in io_pin.type:
                ioe->clr_bit(io_pin.reg_iopwm, io_pin.pwm_channel)

        pm1 = _ioe_i2c_read8(ioe, io_pin.reg_m1)
        pm2 = _ioe_i2c_read8(ioe, io_pin.reg_m2)

        // Clear the pm1 and pm2 bits
        pm1 &= 255 - (1 << io_pin.pin)
        pm2 &= 255 - (1 << io_pin.pin)

        // Set the new pm1 and pm2 bits according to our gpio_mode
        pm1 |= (gpio_mode >> 1) << io_pin.pin
        pm2 |= (gpio_mode & 0b1) << io_pin.pin

        ioe->_ioe_i2c_write8(io_pin.reg_m1, pm1)
        ioe->_ioe_i2c_write8(io_pin.reg_m2, pm2)

        // Set up Schmitt trigger mode on inputs
        if mode in [PIN_MODE_PU, PIN_MODE_IN]:
            ioe->change_bit(io_pin.reg_ps, io_pin.pin, schmitt_trigger)

        // 5th bit of mode encodes default output pin state
        ioe->_ioe_i2c_write8(io_pin.reg_p, (initial_state << 3) | io_pin.pin)

    def input(self, pin, adc_timeout=1):
        /** Read the IO pin state.

        Returns a 12-bit ADC reading if the pin is in ADC mode
        Returns True/False if the pin is in any other input mode
        Returns None if the pin is in PWM mode

        :param adc_timeout: Timeout (in seconds) for an ADC read (default 1.0)

         */
        if pin < 1 or pin > sizeof(ioe->_pins / sizeof(ioe->_pins[0]):
            raise ValueError("Pin should be in range 1-14.")

        io_pin = ioe->_pins[pin - 1]

        if io_pin.mode == PIN_MODE_ADC:
            if ioe->_debug:
                print("Reading ADC from pin {}".format(pin))
            ioe->clr_bits(REG_ADCCON0, 0x0f)
            ioe->set_bits(REG_ADCCON0, io_pin.adc_channel)
            ioe->_ioe_i2c_write8(REG_AINDIDS, 0)
            ioe->set_bit(REG_AINDIDS, io_pin.adc_channel)
            ioe->set_bit(REG_ADCCON1, 0)

            ioe->clr_bit(REG_ADCCON0, 7)  // ADCF - Clear the conversion complete flag
            ioe->set_bit(REG_ADCCON0, 6)  // ADCS - Set the ADC conversion start flag

            // Wait for the ADCF conversion complete flag to be set
            t_start = time.time()
            while not ioe->get_bit(REG_ADCCON0, 7):
                time.sleep(0.01)
                if time.time() - t_start >= adc_timeout:
                    raise RuntimeError("Timeout waiting for ADC conversion!")

            hi = _ioe_i2c_read8(ioe, REG_ADCRH)
            lo = _ioe_i2c_read8(ioe, REG_ADCRL)
            return ((hi << 4) | lo) / 4095.0 * ioe->_vref

        else:
            if ioe->_debug:
                print("Reading IO from pin {}".format(pin))
            pv = ioe->get_bit(io_pin.reg_p, io_pin.pin)

            return HIGH if pv else LOW

    def output(self, pin, value):
        /** Write an IO pin state or PWM duty cycle.

        :param value: Either True/False for OUT, or a number between 0 and PWM period for PWM.

         */
        if pin < 1 or pin > sizeof(ioe->_pins / sizeof(ioe->_pins[0]):
            raise ValueError("Pin should be in range 1-14.")

        io_pin = ioe->_pins[pin - 1]

        if io_pin.mode == PIN_MODE_PWM:
            if ioe->_debug:
                print("Outputting PWM to pin: {pin}".format(pin=pin))
            ioe->_ioe_i2c_write8(io_pin.reg_pwml, value & 0xff)
            ioe->_ioe_i2c_write8(io_pin.reg_pwmh, value >> 8)
            ioe->_pwm_load()

        else:
            if value == LOW:
                if ioe->_debug:
                    print("Outputting LOW to pin: {pin}".format(pin=pin, value=value))
                ioe->clr_bit(io_pin.reg_p, io_pin.pin)
            elif value == HIGH:
                if ioe->_debug:
                    print("Outputting HIGH to pin: {pin}".format(pin=pin, value=value))
                ioe->set_bit(io_pin.reg_p, io_pin.pin)
