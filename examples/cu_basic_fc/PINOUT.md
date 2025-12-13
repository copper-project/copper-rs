# MicoAir743 Pinout

| Peripheral | Bus / Role | Signal | MCU Pin | Betaflight Resource |
| --- | --- | --- | --- | --- |
| IMU – BMI270 | SPI2 | MOSI | PC3 | SPI_SDO2 |
|  |  | MISO | PC2 | SPI_SDI2 |
|  |  | SCLK | PD3 | SPI_SCK2 |
|  |  | CS (BMI270_CS) | PA15 | GYRO_CS1 |
| IMU – BMI088 | SPI2 | MOSI | PC3 | SPI_SDO2 |
|  |  | MISO | PC2 | SPI_SDI2 |
|  |  | SCLK | PD3 | SPI_SCK2 |
|  |  | CS (GYRO) | PD5 | PINIO2 |
|  |  | CS (ACCEL) | PD4 | PINIO1 |
| Barometer – DPS310 | I2C2 | SCL | PB10 | I2C_SCL2 |
|  |  | SDA | PB11 | I2C_SDA2 |
| Compass – IST8310 | I2C2 | SCL | PB10 | I2C_SCL2 |
|  |  | SDA | PB11 | I2C_SDA2 |
| External I2C | I2C1 | SCL | PB6 | I2C_SCL1 |
|  |  | SDA | PB7 | I2C_SDA1 |
| OSD – AT7456E | SPI1 | MOSI | PA7 | SPI_SDO1 |
|  |  | MISO | PA6 | SPI_SDI1 |
|  |  | SCLK | PA5 | SPI_SCK1 |
|  |  | CS (OSD_CS) | PB12 | PREINIT2 |
| USB | USB | DM | PA11 | USB_DM |
|  |  | DP | PA12 | USB_DP |
| MicroSD | SDMMC1 | D0 | PC8 | SDIO_D0 |
|  |  | D1 | PC9 | SDIO_D1 |
|  |  | D2 | PC10 | SDIO_D2 |
|  |  | D3 | PC11 | SDIO_D3 |
|  |  | CLK | PC12 | SDIO_CK |
|  |  | CMD | PD2 | SDIO_CMD |
| UART1 | UART | TX | PA9 | SERIAL_TX1 |
|  |  | RX | PA10 | SERIAL_RX1 |
| UART2 (VTX‑HD) | UART | TX | PA2 | SERIAL_TX2 |
|  |  | RX | PA3 | SERIAL_RX2 |
| UART3 (GPS) | UART | TX | PD8 | (not assigned) |
|  |  | RX | PD9 | (not assigned) |
| UART4 | UART | TX | PA0 | (not assigned) |
|  |  | RX | PA1 | (not assigned) |
| UART6 (RCIN) | UART | TX | PC6 | SERIAL_TX6 |
|  |  | RX | PC7 | SERIAL_RX6 |
| UART7 (ESC Telemetry) | UART | RX | PE7 | (not assigned) |
| UART8 | UART | TX | PE1 | (not assigned) |
|  |  | RX | PE0 | (not assigned) |
| PWM – Motor 1 | TIM1 | CH4 | PE14 | MOTOR1 |
| PWM – Motor 2 | TIM1 | CH3 | PE13 | MOTOR2 |
| PWM – Motor 3 | TIM1 | CH2 | PE11 | MOTOR3 |
| PWM – Motor 4 | TIM1 | CH1 | PE9 | MOTOR4 |
| PWM – Motor 5 | TIM3 | CH4 | PB1 | (free) |
| PWM – Motor 6 | TIM3 | CH3 | PB0 | (free) |
| PWM – Motor 7 | TIM4 | CH1 | PD12 | (free) |
| PWM – Motor 8 | TIM4 | CH2 | PD13 | (free) |
| PWM – Motor 9 | TIM4 | CH3 | PD14 | LED_STRIP |
| PWM – Motor 10 | TIM4 | CH4 | PD15 | BEEPER |
| Status LEDs | GPIO | RED | PE5 | LED3 |
|  |  | BLUE | PE4 | LED1 |
|  |  | GREEN | PE6 | LED2 |
| LED Strip | GPIO | DATA | PD14 | LED_STRIP |
| Beeper | GPIO | BEEP | PD15 | BEEPER |
| Battery sense | ADC | Voltage | PC0 | ADC_BATT |
|  |  | Current | PC1 | ADC_CURR |
| Debug | SWD | SWDIO | PA13 | SWDIO |
|  |  | SWCLK | PA14 | SWCLK |
| Oscillator | 8MHz | OSC_IN | PH0 | (free) |
|  |  | OSC_OUT | PH1 | (free) |
