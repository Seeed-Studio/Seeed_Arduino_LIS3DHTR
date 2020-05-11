# Seeed_Arduino_LIS3DHTR  [![Build Status](https://travis-ci.com/Seeed-Studio/Seeed_Arduino_LIS3DHTR.svg?branch=master)](https://travis-ci.com/Seeed-Studio/Seeed_Arduino_LIS3DHTR)
## Introduction 
An Arduino library for 3-Axis Digital Accelerometer ±2g to 16g (LIS3DHTR).Acceleration data can be obtained using IIC interface and SPI interface.
## How to install  Arduino Library
please refer [here](https://wiki.seeedstudio.com/How_to_install_Arduino_Library/).

## Usage

```C++
// This example use I2C.
#include "LIS3DHTR.h"
#include <Wire.h>
LIS3DHTR<TwoWire> LIS; //IIC
#define WIRE Wire

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  };
  LIS.begin(WIRE, LIS3DHTR_ADDRESS_UPDATED); //IIC init
  delay(100);
  LIS.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);

}
void loop()
{
  if (!LIS)
  {
    Serial.println("LIS3DHTR didn't connect.");
    while (1)
      ;
    return;
  }
  //3 axis
   Serial.print("x:"); Serial.print(LIS.getAccelerationX()); Serial.print("  ");
   Serial.print("y:"); Serial.print(LIS.getAccelerationY()); Serial.print("  ");
   Serial.print("z:"); Serial.println(LIS.getAccelerationZ());

}

```

## API Reference

- **begin(*comm\<TwoWire\>, address\<uint8_t\>=0x18*)** :  Init device
```C++
LIS3DHTR<TwoWire> LIS; //IIC
LIS.begin(Wire, 0x19)
```

- **begin(*comm\<SPIClass\>, sspin\<uint8_t\>=SS*)** :  Init device
```C++
LIS3DHTR<SPIClass> LIS; //SPI
LIS.begin(SPI, 10); //SPI SS/CS
```

----
## License
This software is written by seeed studio<br>
and is licensed under [The MIT License](http://opensource.org/licenses/mit-license.php). Check License.txt for more information.<br>

Contributing to this software is warmly welcomed. You can do this basically by<br>
[forking](https://help.github.com/articles/fork-a-repo), committing modifications and then [pulling requests](https://help.github.com/articles/using-pull-requests) (follow the links above<br>
for operating guide). Adding change log and your contact into file header is encouraged.<br>
Thanks for your contribution.

Seeed Studio is an open hardware facilitation company based in Shenzhen, China. <br>
Benefiting from local manufacture power and convenient global logistic system, <br>
we integrate resources to serve new era of innovation. Seeed also works with <br>
global distributors and partners to push open hardware movement.<br>