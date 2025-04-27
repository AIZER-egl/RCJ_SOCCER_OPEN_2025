/*!
 * @file Adafruit_BNO055.cpp
 *
 *  @mainpage Adafruit BNO055 Orientation Sensor
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the BNO055 orientation sensor
 *
 *  Designed specifically to work with the Adafruit BNO055 9-DOF Breakout.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2472
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  K.Townsend (Adafruit Industries)
 *
 *  @section license License
 *
 *  MIT license, all text above must be included in any redistribution
 */

#include "compass_classes.h"

#include "../software/binarySerializationData.h"

/*!
 *  @brief  Instantiates a new Adafruit_BNO055 class
 *  @param  sensorID
 *          sensor ID
 *  @param  address
 *          i2c address
 *  @param  theWire
 *          Wire object
 */
Adafruit_BNO055::Adafruit_BNO055(int32_t sensorID, uint8_t address) : dataPtr(nullptr) {
    _sensorID = sensorID;
    _address = address;
}

/*!
 *  @brief  Sets up the HW
 *  @param port
 *          i2c port
 *            [I2C_PORT_0, I2C_PORT_1]
 *  @param baud
 *          baud rate
 *            [100000, 400000]
 *            default is 400000
 *  @param sda
 *          sda pin
 *  @param scl
 *          scl pin
 *  @param  mode
 *          mode values
 *           [OPERATION_MODE_CONFIG,
 *            OPERATION_MODE_ACCONLY,
 *            OPERATION_MODE_MAGONLY,
 *            OPERATION_MODE_GYRONLY,
 *            OPERATION_MODE_ACCMAG,
 *            OPERATION_MODE_ACCGYRO,
 *            OPERATION_MODE_MAGGYRO,
 *            OPERATION_MODE_AMG,
 *            OPERATION_MODE_IMUPLUS,
 *            OPERATION_MODE_COMPASS,
 *            OPERATION_MODE_M4G,
 *            OPERATION_MODE_NDOF_FMC_OFF,
 *            OPERATION_MODE_NDOF]
 *  @return true if process is successful
 */
bool Adafruit_BNO055::begin(bool port, uint32_t baud, uint8_t sda, uint8_t scl, BinarySerializationData& data, adafruit_bno055_opmode_t mode) {
    if (port == I2C_PORT_0) {
        _port = i2c0;
    } else if (port == I2C_PORT_1) {
        _port = i2c1;
    }

    i2c_init(i2c0, baud);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);

    uint8_t id = read8(BNO055_CHIP_ID_ADDR);
    std::cout << id << "\n";
    if (id != BNO055_ID) {
        std::cout << "No BNO055 detected\n";
    }

    /* Switch to config mode (just in case since this is the default) */
    setMode(OPERATION_MODE_CONFIG);

    /* Reset */
    write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
    /* sleep_ms incrased to 30ms due to power issues https://tinyurl.com/y375z699 */
    sleep_ms(30);
    while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
        sleep_ms(10);
    }
    sleep_ms(50);

    /* Set to normal power mode */
    write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    sleep_ms(10);

    write8(BNO055_PAGE_ID_ADDR, 0);

    /* Set the output units */
    /*
    uint8_t unitsel = (0 << 7) | // Orientation = Android
                      (0 << 4) | // Temperature = Celsius
                      (0 << 2) | // Euler = Degrees
                      (1 << 1) | // Gyro = Rads
                      (0 << 0);  // Accelerometer = m/s^2
    write8(BNO055_UNIT_SEL_ADDR, unitsel);
    */

    /* Configure axis mapping (see section 3.4) */
    /*
    write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
    sleep_ms(10);
    write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
    sleep_ms(10);
    */

    write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
    sleep_ms(10);
    /* Set the requested operating mode (see section 3.3) */
    setMode(mode);
    sleep_ms(20);

    dataPtr = &data;

    return true;
}

bool Adafruit_BNO055::isAlive() {
    uint8_t id = read8(BNO055_CHIP_ID_ADDR);
    return id == BNO055_ID;
}

/*!
 *  @brief  Puts the chip in the specified operating mode
 *  @param  mode
 *          mode values
 *           [OPERATION_MODE_CONFIG,
 *            OPERATION_MODE_ACCONLY,
 *            OPERATION_MODE_MAGONLY,
 *            OPERATION_MODE_GYRONLY,
 *            OPERATION_MODE_ACCMAG,
 *            OPERATION_MODE_ACCGYRO,
 *            OPERATION_MODE_MAGGYRO,
 *            OPERATION_MODE_AMG,
 *            OPERATION_MODE_IMUPLUS,
 *            OPERATION_MODE_COMPASS,
 *            OPERATION_MODE_M4G,
 *            OPERATION_MODE_NDOF_FMC_OFF,
 *            OPERATION_MODE_NDOF]
 */
void Adafruit_BNO055::setMode(adafruit_bno055_opmode_t mode) {
    _mode = mode;
    write8(BNO055_OPR_MODE_ADDR, _mode);
    sleep_ms(30);
}

/*!
 *  @brief  Gets the current operating mode of the chip
 *  @return  operating_mode in integer which can be mapped in Section 3.3
 *           for example: a return of 12 (0X0C) => NDOF
 */
adafruit_bno055_opmode_t Adafruit_BNO055::getMode() {
    return (adafruit_bno055_opmode_t)read8(BNO055_OPR_MODE_ADDR);
}

/*!
 *  @brief  Changes the chip's axis remap
 *  @param  remapcode
 *          remap code possible values
 *          [REMAP_CONFIG_P0
 *           REMAP_CONFIG_P1 (default)
 *           REMAP_CONFIG_P2
 *           REMAP_CONFIG_P3
 *           REMAP_CONFIG_P4
 *           REMAP_CONFIG_P5
 *           REMAP_CONFIG_P6
 *           REMAP_CONFIG_P7]
 */
void Adafruit_BNO055::setAxisRemap(
        adafruit_bno055_axis_remap_config_t remapcode) {
    adafruit_bno055_opmode_t modeback = _mode;

    setMode(OPERATION_MODE_CONFIG);
    sleep_ms(25);
    write8(BNO055_AXIS_MAP_CONFIG_ADDR, remapcode);
    sleep_ms(10);
    /* Set the requested operating mode (see section 3.3) */
    setMode(modeback);
    sleep_ms(20);
}

/*!
 *  @brief  Changes the chip's axis signs
 *  @param  remapsign
 *          remap sign possible values
 *          [REMAP_SIGN_P0
 *           REMAP_SIGN_P1 (default)
 *           REMAP_SIGN_P2
 *           REMAP_SIGN_P3
 *           REMAP_SIGN_P4
 *           REMAP_SIGN_P5
 *           REMAP_SIGN_P6
 *           REMAP_SIGN_P7]
 */
void Adafruit_BNO055::setAxisSign(adafruit_bno055_axis_remap_sign_t remapsign) {
    adafruit_bno055_opmode_t modeback = _mode;

    setMode(OPERATION_MODE_CONFIG);
    sleep_ms(25);
    write8(BNO055_AXIS_MAP_SIGN_ADDR, remapsign);
    sleep_ms(10);
    /* Set the requested operating mode (see section 3.3) */
    setMode(modeback);
    sleep_ms(20);
}

/*!
 *  @brief  Use the external 32.768KHz crystal
 *  @param  usextal
 *          use external crystal boolean
 */
void Adafruit_BNO055::setExtCrystalUse(bool usextal) {
    adafruit_bno055_opmode_t modeback = _mode;

    /* Switch to config mode (just in case since this is the default) */
    setMode(OPERATION_MODE_CONFIG);
    sleep_ms(25);
    write8(BNO055_PAGE_ID_ADDR, 0);
    if (usextal) {
        write8(BNO055_SYS_TRIGGER_ADDR, 0x80);
    } else {
        write8(BNO055_SYS_TRIGGER_ADDR, 0x00);
    }
    sleep_ms(10);
    /* Set the requested operating mode (see section 3.3) */
    setMode(modeback);
    sleep_ms(20);
}

/*!
 *   @brief  Gets the latest system status info
 *   @param  system_status
 *           system status info
 *   @param  self_test_result
 *           self test result
 *   @param  system_error
 *           system error info
 */
void Adafruit_BNO055::getSystemStatus(uint8_t *system_status,
                                      uint8_t *self_test_result,
                                      uint8_t *system_error) {
    write8(BNO055_PAGE_ID_ADDR, 0);

    /* System Status (see section 4.3.58)
       0 = Idle
       1 = System Error
       2 = Initializing Peripherals
       3 = System Iniitalization
       4 = Executing Self-Test
       5 = Sensor fusio algorithm running
       6 = System running without fusion algorithms
     */

    if (system_status != 0)
        *system_status = read8(BNO055_SYS_STAT_ADDR);

    /* Self Test Results
       1 = test passed, 0 = test failed

       Bit 0 = Accelerometer self test
       Bit 1 = Magnetometer self test
       Bit 2 = Gyroscope self test
       Bit 3 = MCU self test

       0x0F = all good!
     */

    if (self_test_result != 0)
        *self_test_result = read8(BNO055_SELFTEST_RESULT_ADDR);

    /* System Error (see section 4.3.59)
       0 = No error
       1 = Peripheral initialization error
       2 = System initialization error
       3 = Self test result failed
       4 = Register map value out of range
       5 = Register map address out of range
       6 = Register map write error
       7 = BNO low power mode not available for selected operat ion mode
       8 = Accelerometer power mode not available
       9 = Fusion algorithm configuration error
       A = Sensor configuration error
     */

    if (system_error != 0)
        *system_error = read8(BNO055_SYS_ERR_ADDR);

    sleep_ms(200);
}

/*!
 *  @brief  Gets the chip revision numbers
 *  @param  info
 *          revision info
 */
void Adafruit_BNO055::getRevInfo(adafruit_bno055_rev_info_t *info) {
    uint8_t a, b;

    std::memset(info, 0, sizeof(adafruit_bno055_rev_info_t));

    /* Check the accelerometer revision */
    info->accel_rev = read8(BNO055_ACCEL_REV_ID_ADDR);

    /* Check the magnetometer revision */
    info->mag_rev = read8(BNO055_MAG_REV_ID_ADDR);

    /* Check the gyroscope revision */
    info->gyro_rev = read8(BNO055_GYRO_REV_ID_ADDR);

    /* Check the SW revision */
    info->bl_rev = read8(BNO055_BL_REV_ID_ADDR);

    a = read8(BNO055_SW_REV_ID_LSB_ADDR);
    b = read8(BNO055_SW_REV_ID_MSB_ADDR);
    info->sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
}

/*!
 *  @brief  Gets current calibration state.  Each value should be a uint8_t
 *          pointer and it will be set to 0 if not calibrated and 3 if
 *          fully calibrated.
 *          See section 34.3.54
 *  @param  sys
 *          Current system calibration status, depends on status of all sensors,
 * read-only
 *  @param  gyro
 *          Current calibration status of Gyroscope, read-only
 *  @param  accel
 *          Current calibration status of Accelerometer, read-only
 *  @param  mag
 *          Current calibration status of Magnetometer, read-only
 */
void Adafruit_BNO055::getCalibration(uint8_t *sys, uint8_t *gyro,
                                     uint8_t *accel, uint8_t *mag) {
    uint8_t calData = read8(BNO055_CALIB_STAT_ADDR);
    if (sys != NULL) {
        *sys = (calData >> 6) & 0x03;
    }
    if (gyro != NULL) {
        *gyro = (calData >> 4) & 0x03;
    }
    if (accel != NULL) {
        *accel = (calData >> 2) & 0x03;
    }
    if (mag != NULL) {
        *mag = calData & 0x03;
    }
}

/*!
 *  @brief  Gets the temperature in degrees celsius
 *  @return temperature in degrees celsius
 */
int8_t Adafruit_BNO055::getTemp() {
    int8_t temp = (int8_t)(read8(BNO055_TEMP_ADDR));
    return temp;
}

/*!
 *  @brief   Gets a vector reading from the specified source
 *  @param   vector_type
 *           possible vector type values
 *           [VECTOR_ACCELEROMETER
 *            VECTOR_MAGNETOMETER
 *            VECTOR_GYROSCOPE
 *            VECTOR_EULER
 *            VECTOR_LINEARACCEL
 *            VECTOR_GRAVITY]
 *  @return  vector from specified source
 */
std::vector<double> Adafruit_BNO055::getVector(adafruit_vector_type_t vector_type) {
    std::vector<double> xyz(3);
    uint8_t buffer[6];
    memset(buffer, 0, 6); // Always make sure to clear the buffer to start with

    int16_t x, y, z;
    x = y = z = 0;

    /* Read vector data (6 bytes) */
    readLen((adafruit_bno055_reg_t)vector_type, buffer, 6);

    x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
    y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
    z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

    /*!
     * Convert the value to an appropriate range (section 3.6.4)
     * and assign the value to the Vector type
     */
    switch (vector_type) {
        case VECTOR_MAGNETOMETER:
            /* 1uT = 16 LSB */
            xyz[0] = ((double)x) / 16.0;
            xyz[1] = ((double)y) / 16.0;
            xyz[2] = ((double)z) / 16.0;
            break;
        case VECTOR_GYROSCOPE:
            /* 1dps = 16 LSB */
            xyz[0] = ((double)x) / 16.0;
            xyz[1] = ((double)y) / 16.0;
            xyz[2] = ((double)z) / 16.0;
            break;
        case VECTOR_EULER:
            /* 1 degree = 16 LSB */
            xyz[0] = ((double)x) / 16.0;
            xyz[1] = ((double)y) / 16.0;
            xyz[2] = ((double)z) / 16.0;
            break;
        case VECTOR_ACCELEROMETER:
            /* 1m/s^2 = 100 LSB */
            xyz[0] = ((double)x) / 100.0;
            xyz[1] = ((double)y) / 100.0;
            xyz[2] = ((double)z) / 100.0;
            break;
        case VECTOR_LINEARACCEL:
            /* 1m/s^2 = 100 LSB */
            xyz[0] = ((double)x) / 100.0;
            xyz[1] = ((double)y) / 100.0;
            xyz[2] = ((double)z) / 100.0;
            break;
        case VECTOR_GRAVITY:
            /* 1m/s^2 = 100 LSB */
            xyz[0] = ((double)x) / 100.0;
            xyz[1] = ((double)y) / 100.0;
            xyz[2] = ((double)z) / 100.0;
            break;
    }

    return xyz;
}

/*!
 *  @brief  Gets a quaternion reading from the specified source
 *  @return quaternion reading
 */
std::vector<double> Adafruit_BNO055::getQuat() {
    uint8_t buffer[8];
    memset(buffer, 0, 8);

    int16_t x, y, z, w;
    x = y = z = w = 0;

    /* Read quat data (8 bytes) */
    readLen(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
    w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
    x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
    y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
    z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

    /*!
     * Assign to Quaternion
     * See
     * https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
     * 3.6.5.5 Orientation (Quaternion)
     */
    const double scale = (1.0 / (1 << 14));
    std::vector<double> quat = {scale * w, scale * x, scale * y, scale * z};
    return quat;
}

/*!
 *  @brief  Reads the sensor's offset registers into a byte array
 *  @param  calibData
 *          Calibration offset (buffer size should be 22)
 *  @return true if read is successful
 */
bool Adafruit_BNO055::getSensorOffsets(uint8_t *calibData) {
    if (isFullyCalibrated()) {
        adafruit_bno055_opmode_t lastMode = _mode;
        setMode(OPERATION_MODE_CONFIG);

        readLen(ACCEL_OFFSET_X_LSB_ADDR, calibData, NUM_BNO055_OFFSET_REGISTERS);

        setMode(lastMode);
        return true;
    }
    return false;
}

/*!
 *  @brief  Reads the sensor's offset registers into an offset struct
 *  @param  offsets_type
 *          type of offsets
 *  @return true if read is successful
 */
bool Adafruit_BNO055::getSensorOffsets(
        adafruit_bno055_offsets_t &offsets_type) {
    if (isFullyCalibrated()) {
        adafruit_bno055_opmode_t lastMode = _mode;
        setMode(OPERATION_MODE_CONFIG);
        sleep_ms(25);

        /* Accel offset range depends on the G-range:
           +/-2g  = +/- 2000 mg
           +/-4g  = +/- 4000 mg
           +/-8g  = +/- 8000 mg
           +/-1§g = +/- 16000 mg */
        offsets_type.accel_offset_x = (read8(ACCEL_OFFSET_X_MSB_ADDR) << 8) |
                                      (read8(ACCEL_OFFSET_X_LSB_ADDR));
        offsets_type.accel_offset_y = (read8(ACCEL_OFFSET_Y_MSB_ADDR) << 8) |
                                      (read8(ACCEL_OFFSET_Y_LSB_ADDR));
        offsets_type.accel_offset_z = (read8(ACCEL_OFFSET_Z_MSB_ADDR) << 8) |
                                      (read8(ACCEL_OFFSET_Z_LSB_ADDR));

        /* Magnetometer offset range = +/- 6400 LSB where 1uT = 16 LSB */
        offsets_type.mag_offset_x =
                (read8(MAG_OFFSET_X_MSB_ADDR) << 8) | (read8(MAG_OFFSET_X_LSB_ADDR));
        offsets_type.mag_offset_y =
                (read8(MAG_OFFSET_Y_MSB_ADDR) << 8) | (read8(MAG_OFFSET_Y_LSB_ADDR));
        offsets_type.mag_offset_z =
                (read8(MAG_OFFSET_Z_MSB_ADDR) << 8) | (read8(MAG_OFFSET_Z_LSB_ADDR));

        /* Gyro offset range depends on the DPS range:
          2000 dps = +/- 32000 LSB
          1000 dps = +/- 16000 LSB
           500 dps = +/- 8000 LSB
           250 dps = +/- 4000 LSB
           125 dps = +/- 2000 LSB
           ... where 1 DPS = 16 LSB */
        offsets_type.gyro_offset_x =
                (read8(GYRO_OFFSET_X_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_X_LSB_ADDR));
        offsets_type.gyro_offset_y =
                (read8(GYRO_OFFSET_Y_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_Y_LSB_ADDR));
        offsets_type.gyro_offset_z =
                (read8(GYRO_OFFSET_Z_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_Z_LSB_ADDR));

        /* Accelerometer radius = +/- 1000 LSB */
        offsets_type.accel_radius =
                (read8(ACCEL_RADIUS_MSB_ADDR) << 8) | (read8(ACCEL_RADIUS_LSB_ADDR));

        /* Magnetometer radius = +/- 960 LSB */
        offsets_type.mag_radius =
                (read8(MAG_RADIUS_MSB_ADDR) << 8) | (read8(MAG_RADIUS_LSB_ADDR));

        setMode(lastMode);
        return true;
    }
    return false;
}

/*!
 *  @brief  Writes an array of calibration values to the sensor's offset
 *  @param  calibData
 *          calibration data
 */
void Adafruit_BNO055::setSensorOffsets(const uint8_t *calibData) {
    adafruit_bno055_opmode_t lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    sleep_ms(25);

    /* Note: Configuration will take place only when user writes to the last
       byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
       Therefore the last byte must be written whenever the user wants to
       changes the configuration. */

    /* A writeLen() would make this much cleaner */
    write8(ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);
    write8(ACCEL_OFFSET_X_MSB_ADDR, calibData[1]);
    write8(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]);
    write8(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]);
    write8(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]);
    write8(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]);

    write8(MAG_OFFSET_X_LSB_ADDR, calibData[6]);
    write8(MAG_OFFSET_X_MSB_ADDR, calibData[7]);
    write8(MAG_OFFSET_Y_LSB_ADDR, calibData[8]);
    write8(MAG_OFFSET_Y_MSB_ADDR, calibData[9]);
    write8(MAG_OFFSET_Z_LSB_ADDR, calibData[10]);
    write8(MAG_OFFSET_Z_MSB_ADDR, calibData[11]);

    write8(GYRO_OFFSET_X_LSB_ADDR, calibData[12]);
    write8(GYRO_OFFSET_X_MSB_ADDR, calibData[13]);
    write8(GYRO_OFFSET_Y_LSB_ADDR, calibData[14]);
    write8(GYRO_OFFSET_Y_MSB_ADDR, calibData[15]);
    write8(GYRO_OFFSET_Z_LSB_ADDR, calibData[16]);
    write8(GYRO_OFFSET_Z_MSB_ADDR, calibData[17]);

    write8(ACCEL_RADIUS_LSB_ADDR, calibData[18]);
    write8(ACCEL_RADIUS_MSB_ADDR, calibData[19]);

    write8(MAG_RADIUS_LSB_ADDR, calibData[20]);
    write8(MAG_RADIUS_MSB_ADDR, calibData[21]);

    setMode(lastMode);
}

/*!
 *  @brief  Writes to the sensor's offset registers from an offset struct
 *  @param  offsets_type
 *          accel_offset_x = acceleration offset x
 *          accel_offset_y = acceleration offset y
 *          accel_offset_z = acceleration offset z
 *
 *          mag_offset_x   = magnetometer offset x
 *          mag_offset_y   = magnetometer offset y
 *          mag_offset_z   = magnetometer offset z
 *
 *          gyro_offset_x  = gyroscrope offset x
 *          gyro_offset_y  = gyroscrope offset y
 *          gyro_offset_z  = gyroscrope offset z
 */
void Adafruit_BNO055::setSensorOffsets(
        const adafruit_bno055_offsets_t &offsets_type) {
    adafruit_bno055_opmode_t lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    sleep_ms(25);

    /* Note: Configuration will take place only when user writes to the last
       byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
       Therefore the last byte must be written whenever the user wants to
       changes the configuration. */

    write8(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type.accel_offset_x) & 0x0FF);
    write8(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type.accel_offset_x >> 8) & 0x0FF);
    write8(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type.accel_offset_y) & 0x0FF);
    write8(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type.accel_offset_y >> 8) & 0x0FF);
    write8(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type.accel_offset_z) & 0x0FF);
    write8(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type.accel_offset_z >> 8) & 0x0FF);

    write8(MAG_OFFSET_X_LSB_ADDR, (offsets_type.mag_offset_x) & 0x0FF);
    write8(MAG_OFFSET_X_MSB_ADDR, (offsets_type.mag_offset_x >> 8) & 0x0FF);
    write8(MAG_OFFSET_Y_LSB_ADDR, (offsets_type.mag_offset_y) & 0x0FF);
    write8(MAG_OFFSET_Y_MSB_ADDR, (offsets_type.mag_offset_y >> 8) & 0x0FF);
    write8(MAG_OFFSET_Z_LSB_ADDR, (offsets_type.mag_offset_z) & 0x0FF);
    write8(MAG_OFFSET_Z_MSB_ADDR, (offsets_type.mag_offset_z >> 8) & 0x0FF);

    write8(GYRO_OFFSET_X_LSB_ADDR, (offsets_type.gyro_offset_x) & 0x0FF);
    write8(GYRO_OFFSET_X_MSB_ADDR, (offsets_type.gyro_offset_x >> 8) & 0x0FF);
    write8(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type.gyro_offset_y) & 0x0FF);
    write8(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type.gyro_offset_y >> 8) & 0x0FF);
    write8(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type.gyro_offset_z) & 0x0FF);
    write8(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type.gyro_offset_z >> 8) & 0x0FF);

    write8(ACCEL_RADIUS_LSB_ADDR, (offsets_type.accel_radius) & 0x0FF);
    write8(ACCEL_RADIUS_MSB_ADDR, (offsets_type.accel_radius >> 8) & 0x0FF);

    write8(MAG_RADIUS_LSB_ADDR, (offsets_type.mag_radius) & 0x0FF);
    write8(MAG_RADIUS_MSB_ADDR, (offsets_type.mag_radius >> 8) & 0x0FF);

    setMode(lastMode);
}

/*!
 *  @brief  Checks of all cal status values are set to 3 (fully calibrated)
 *  @return status of calibration
 */
bool Adafruit_BNO055::isFullyCalibrated() {
    uint8_t system, gyro, accel, mag;
    getCalibration(&system, &gyro, &accel, &mag);

    switch (_mode) {
        case OPERATION_MODE_ACCONLY:
            return (accel == 3);
        case OPERATION_MODE_MAGONLY:
            return (mag == 3);
        case OPERATION_MODE_GYRONLY:
        case OPERATION_MODE_M4G: /* No magnetometer calibration required. */
            return (gyro == 3);
        case OPERATION_MODE_ACCMAG:
        case OPERATION_MODE_COMPASS:
            return (accel == 3 && mag == 3);
        case OPERATION_MODE_ACCGYRO:
        case OPERATION_MODE_IMUPLUS:
            return (accel == 3 && gyro == 3);
        case OPERATION_MODE_MAGGYRO:
            return (mag == 3 && gyro == 3);
        default:
            return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
    }
}

/*!
 *  @brief  Enter Suspend mode (i.e., sleep)
 */
void Adafruit_BNO055::enterSuspendMode() {
    adafruit_bno055_opmode_t modeback = _mode;

    /* Switch to config mode (just in case since this is the default) */
    setMode(OPERATION_MODE_CONFIG);
    sleep_ms(25);
    write8(BNO055_PWR_MODE_ADDR, 0x02);
    /* Set the requested operating mode (see section 3.3) */
    setMode(modeback);
    sleep_ms(20);
}

/*!
 *  @brief  Enter Normal mode (i.e., wake)
 */
void Adafruit_BNO055::enterNormalMode() {
    adafruit_bno055_opmode_t modeback = _mode;

    /* Switch to config mode (just in case since this is the default) */
    setMode(OPERATION_MODE_CONFIG);
    sleep_ms(25);
    write8(BNO055_PWR_MODE_ADDR, 0x00);
    /* Set the requested operating mode (see section 3.3) */
    setMode(modeback);
    sleep_ms(20);
}

/*!
 *  @brief  Writes an 8 bit value over I2C
 */
bool Adafruit_BNO055::write8(adafruit_bno055_reg_t reg, uint8_t value) {
    uint8_t buffer[2] = {(uint8_t)reg, (uint8_t)value};
    i2c_write_blocking(_port, _address, buffer, 2, false);
    return true;
}

/*!
 *  @brief  Reads an 8 bit value over I2C
 */
uint8_t Adafruit_BNO055::read8(adafruit_bno055_reg_t reg) {
    uint8_t buffer[1] = {reg};
    i2c_write_blocking(_port, _address, buffer, 1, false);
    i2c_read_blocking(_port, _address, buffer, 1, false);
    return buffer[0];
}

/*!
 *  @brief  Reads the specified number of bytes over I2C
 */
bool Adafruit_BNO055::readLen(adafruit_bno055_reg_t reg, uint8_t *buffer,
                              uint8_t len) {
    uint8_t reg_buf[1] = {(uint8_t)reg};
    i2c_write_blocking(_port, _address, reg_buf, 1, false);
    return i2c_read_blocking(_port, _address, buffer, len, false);
}

void Adafruit_BNO055::setYawOffset(float new_yaw_offset) {
    yaw_offset = new_yaw_offset;
}


void Adafruit_BNO055::tick() {
    if (!dataPtr) return;

    if ((time_us_64() / 1000) - previous_measure >= 25) {
        yaw = getYaw();

        yaw = yaw - yaw_offset;

        if (yaw < 0) {
            yaw += 360;
        }
        if (yaw > 360) {
            yaw -= 360;
        }

        if (static_cast<int16_t>(yaw) == 10) {
            yaw = 11;
        }

        yaw = (yaw > 180) ? (yaw - 360) : yaw;

        dataPtr -> compass_yaw = static_cast<int16_t>(yaw);

        previous_measure = time_us_64() / 1000;
    }
}

float Adafruit_BNO055::getYaw() {
    float v = getVector(VECTOR_EULER)[0];
    v = (v > 180.0) ? (v - 360.0) : v;
    raw_yaw = v;
    return v;
}

float Adafruit_BNO055::getPitch() {
    return getVector(VECTOR_EULER)[1];
}

float Adafruit_BNO055::getRoll() {
    return getVector(VECTOR_EULER)[2];
}