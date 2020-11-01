# ADS1220.h
A single-header pure-C driver for the ADS1220 24-bit ADC using STM32's HAL

Usage(internal reference, PGA=1, two differential inputs, default sampling speed):
```C++
ADS1220_regs regs = ADS1220_default_regs;
uint8_t ini = ADS1220_init(&hspi1, &regs); // Optionally check for failure
HAL_Delay(100); // Best to let settle
ADS1220_set_pga_gain(&hspi1, ADS1220_PGA_GAIN_1, &regs);
ADS1220_set_conv_mode_single_shot(&hspi1, &regs);
int32_t ch1 = ADS1220_read_singleshot_channel(&hspi1, ADS1220_MUX_AIN0_AIN1, &regs, ADS1220_DRDY_GPIO_Port, ADS1220_DRDY_Pin, 100);
int32_t ch2 = ADS1220_read_singleshot_channel(&hspi1, ADS1220_MUX_AIN2_AIN3, &regs, ADS1220_DRDY_GPIO_Port, ADS1220_DRDY_Pin, 100);
```
