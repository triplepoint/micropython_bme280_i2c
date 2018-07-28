# Introduction
This Micropython module enables I2C communication with a Bosch BME280 temperature, humidity, and pressure sensor.

# Usage
This module pretty closely follows the Bosch reference library's behavior (see the references below).

The basic operation of the module requires initialization of an `BME280_I2C` instance, followed by sensor configuration, and finally acquiring a measurement.

Note that the BME280 has a couple of different operating modes (FORCED, and NORMAL), as well as several oversampling and filtering options, and it's a good idea to understand these specifics in order to get the most out of the part.  See the data sheet, specifically section `3. Functional description` to get an understanding of how this sensor works.

Also, in the examples below, the I2C address is supplied as `bme280_i2c.BME280_I2C_ADDR_SEC` (`0x77`).  Be aware that `bme280_i2c.BME280_I2C_ADDR_PRIM` (`0x76`) is also available.

## Available Methods
### `get_measurement_settings()`
Returns a dict with the sensor's currently-configured settings for the filter coefficient, standby time, and the oversampling settings for each of humidity, pressure, and temperature.  The result would look like:
``` python
sensor.get_measurement_settings()

{
    'filter': 0,
    'standby_time': 0,
    'osr_h': 1,
    'osr_p': 1,
    'osr_t': 1
}
```
Where the values are constants that represent the values of the various settings.  See the various `BME280_OVERSAMPLING_*`, `BME280_STANDBY_TIME_*`, and `BME280_FILTER_COEFF_*` defines at the top of `bme280_i2c.py` and listed below.

### `set_measurement_settings(settings: dict)`
Sets the sensor configuration with a dict similar to the one returned by `get_measurement_settings()` above.  Note that all the keys are optional, and leaving one out will retain the currently-set value.

### `get_power_mode()`
Returns the currently set sensor power mode, where the value is one of the `BME280_*_MODE` constants.  

### `set_power_mode(mode: int)`
Set the sensor power mode, where the value is one of the `BME280_*_MODE` constants.  

Be sure to read section 3.3.1 of the data sheet, and understand that setting the power mode to FORCED will immediately return the sensor to the SLEEP mode, after taking a single sample.

### `get_measurement()`
Acquire and return a dict of the sensor's values, like:
``` python
sensor.get_measurement()

{
    'temperature': 27.86,
    'pressure': 101412.0,
    'humidity': 39.5
}
```
Where the values are in Degrees Celsius, Pascals, and % Relative Humidity, respectively.

## Configuration Constants
``` python
# BME280 default address
BME280_I2C_ADDR_PRIM         : 0x76
BME280_I2C_ADDR_SEC          : 0x77

# Sensor Power Mode Options
BME280_SLEEP_MODE            : 0x00
BME280_FORCED_MODE           : 0x01
BME280_NORMAL_MODE           : 0x03

# Oversampling Options
BME280_NO_OVERSAMPLING       : 0x00
BME280_OVERSAMPLING_1X       : 0x01
BME280_OVERSAMPLING_2X       : 0x02
BME280_OVERSAMPLING_4X       : 0x03
BME280_OVERSAMPLING_8X       : 0x04
BME280_OVERSAMPLING_16X      : 0x05

# Standby Duration Options
BME280_STANDBY_TIME_500_US   : 0x00  # Note this is microseconds, so 0.5 ms
BME280_STANDBY_TIME_62_5_MS  : 0x01
BME280_STANDBY_TIME_125_MS   : 0x02
BME280_STANDBY_TIME_250_MS   : 0x03
BME280_STANDBY_TIME_500_MS   : 0x04
BME280_STANDBY_TIME_1000_MS  : 0x05
BME280_STANDBY_TIME_10_MS    : 0x06
BME280_STANDBY_TIME_20_MS    : 0x07

# Filter Coefficient Options
BME280_FILTER_COEFF_OFF      : 0x00
BME280_FILTER_COEFF_2        : 0x01
BME280_FILTER_COEFF_4        : 0x02
BME280_FILTER_COEFF_8        : 0x03
BME280_FILTER_COEFF_16       : 0x04
```

## "Normal" Power Mode Example
Once enabled, Normal mode samples automatically at a given rate for as long as the sensor has power.  This mode makes sense for high sample-rate applications.

This example implements the advised settings from the data sheet section `3.5.3 Indoor navigation`:

``` python
import machine
import bme280_i2c
import time

# Create a micropython I2C object with the appropriate device pins
i2c = machine.I2C(scl=machine.Pin(5), sda=machine.Pin(4))

# Create a sensor object to represent the BME280
# Note that this will error if the device can't be reached over I2C.
sensor = bme280_i2c.BME280_I2C(address=bme280_i2c.BME280_I2C_ADDR_SEC, i2c=i2c)

# Configure the sensor for the application in question.
sensor.set_measurement_settings({
    'filter': bme280_i2c.BME280_FILTER_COEFF_16,
    'standby_time': bme280_i2c.BME280_STANDBY_TIME_500_US,
    'osr_h': bme280_i2c.BME280_OVERSAMPLING_1X,
    'osr_p': bme280_i2c.BME280_OVERSAMPLING_16X,
    'osr_t': bme280_i2c.BME280_OVERSAMPLING_2X})

# Start the sensor automatically sensing
sensor.set_power_mode(bme280_i2c.BME280_NORMAL_MODE)

# Wait for the measurement settle time, print the measurement, and repeat
while 1:
    time.sleep_ms(70)
    print( sensor.get_measurement() )

# The above code repeatedly prints a line like:
# {'pressure': 101412.0, 'humidity': 39.5, 'temperature': 27.86}
```

## "Forced" Power Mode Example
Once enabled, forced mode takes a single measurement and then returns the sensor to sleep mode.  Acquiring a new sample requires another set to forced mode.  This mode is convenient to conserve power for low sample rate applications.

This example implements the advised settings from the data sheet section `3.5.1 Weather monitoring`:

``` python
import machine
import bme280_i2c
import time

i2c = machine.I2C(scl=machine.Pin(5), sda=machine.Pin(4))

sensor = bme280_i2c.BME280_I2C(address=bme280_i2c.BME280_I2C_ADDR_SEC, i2c=i2c)

sensor.set_measurement_settings({
    'filter': bme280_i2c.BME280_FILTER_COEFF_OFF,
    'osr_h': bme280_i2c.BME280_OVERSAMPLING_1X,
    'osr_p': bme280_i2c.BME280_OVERSAMPLING_1X,
    'osr_t': bme280_i2c.BME280_OVERSAMPLING_1X})

while 1:
    sensor.set_power_mode(bme280_i2c.BME280_FORCED_MODE)
    time.sleep_ms(40)
    print( sensor.get_measurement() )

# {'pressure': 101412.0, 'humidity': 39.5, 'temperature': 27.86}
```

## A Note About Measurement Duration
In the above examples there are sleep commands issued prior to each measurement to pause a given number of milliseconds before acquiring a new sample.  These pauses are defined by the data sheet in section `9. Appendix B: Measurement time and current calculation`.  

You should read the whole thing to properly understand the determination of delays before sampling, but the basic idea is that the measurement itself has a typical and maximum amount of time to complete, depending on the 3 different oversampling configuration values.

In addition, the Normal power mode has an additional standby time which you can configure, and which adds to the measurement duration.

See the data sheet for the details.  Note that the weird factor of 1000 in their calculations is just to deal with the milliseconds vs seconds conversion.

# Reference Material
- [Bosch BME280 Product Information](https://www.bosch-sensortec.com/bst/products/all_products/bme280)
- [Bosch BME280 Datasheet](https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME280_DS001-12.pdf)
- [Bosch Sensortec Reference Driver (in C)](https://github.com/BoschSensortec/BME280_driver)
