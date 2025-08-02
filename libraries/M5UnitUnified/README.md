# M5UnitUnified(α release)

[日本語](README.ja.md)

**A new approach to connect and handle various M5 units in the M5Stack**  
Library for M5Stack Series and M5Unit Series

**Notice: Now α version**  
Please send your comments and requests to Issue or PR.

## Overview
M5UnitUnified is a library for unified handling of various M5 units products.

### Unified APIs
Each unit's external library has its own API design.  
Unify basic APIs so that all units can be handled in the same way.

### Unified connections and communications
Each unit's external library requires its own communication functions and assumptions.  
Unify prerequisites and communication methods.  
In the future, we plan to work with [M5HAL (Hardware Abstraction Layer)](https://github.com/m5stack/M5HAL) to unified communicatation  with each unit.

### Unified Licensing
External library licenses for each unit are mixed.  
All M5UnitUnified and related libraries are under the [MIT license](LICENSE).


## How to install
Alpha version, but registered in the Library Manager

### Arduino IDE

1. Using library manager and select the library of the unit you want to use (e.g. M5Unit-GESTURE)

Dependent M5UnitUnfied related libraries will be downloaded automatically.

### PlatformIO
1. Write lib\_deps settings to platformio.ini
```ini
lib_deps= m5stack/M5Unit-foo ; Unit to be used
```
Dependent M5UnitUnfied related libraries will be downloaded automatically.


## How to use

See also examples for each unit repositry too.

### UnitComponent with UnitUnified (Standard usage)

Simple example of the UnitCO2  
UnitCO2 is started with default settings in Units.begin(), and loop() print logs measurement data.

```cpp
// If you use other units, change include files(*1), instances(*2), and get values(*3)
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedENV.h>  // *1 Include the header of the unit to be used

m5::unit::UnitUnified Units;
m5::unit::UnitCO2 unit;  // *2 Instance of the unit

void setup() {
    M5.begin();

    auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
    auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
    M5_LOGI("getPin: SDA:%u SCL:%u", pin_num_sda, pin_num_scl);
    Wire.begin(pin_num_sda, pin_num_scl, 400 * 1000U);

    M5.Display.clear(TFT_DARKGREEN);
    if (!Units.add(unit, Wire)  // Add unit to UnitUnified manager
        || !Units.begin()) {    // Begin each unit
        M5_LOGE("Failed to add/begin");
        M5.Display.clear(TFT_RED);
    }
}

void loop() {
    M5.begin();
    Units.update();
    if (unit.updated()) {
        // *3 Obtaining unit-specific measurements
        M5_LOGI("CO2:%u Temp:%f Hum:%f", unit.co2(), unit.temperature(), unit.humidity());
    }
}
```

- Nonstandard usage
  - [To update the unit yourself usage example](examples/Basic/SelfUpdate)
  - [Using only unit component without UnitUnified manager](examples/Basic/ComponentOnly)

## Supported things
### Supported frameworks
- Arduino

Support ESP-IDF with M5HAL in the future.

### Supported connection
- I2C with TwoWire
- GPIO (Currently only functions required for the units are included)

Support UART in the future.

### Supported devices, units
See also [Wiki](https://github.com/m5stack/M5UnitUnified/wiki/)

## Examples
For exampless of each unit, please refer to the respective unit's repository.  
[The examples in this repository](examples/Basic) are for M5UnitUnified in general

## Doxygen document

[GitHub Pages](https://m5stack.github.io/M5UnitUnified/)

If you want to generate documents on your local machine, execute the following command

```
bash docs/doxy.sh
```

It will output it under docs/html  
If you want to output Git commit hashes to html, do it for the git cloned folder.

### Required
- [Doxyegn](https://www.doxygen.nl/)
- [pcregrep](https://formulae.brew.sh/formula/pcre2)
- [Git](https://git-scm.com/) (Output commit hash to html)

