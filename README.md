# f7_adc_tempSensor
STM32F7xx Read Interal Temperature Sensor ADC<br>
Read Internal Temperature Sensor of STM32F769I Discovery using stm32 HAL and LL (low-level) drivers<br>
See section 5.3.25 of the data sheet for STM32F765xx STM32F767xx STM32F768Ax STM32F769xx<br>
Outputs to UART One which is stlink virtual com port<br>
<br>
To use with other chips, change:<br>
VREFINT_CAL_ADDR<br>
VREFINT_CAL_VREF<br>
TEMPSENSOR_CAL1_ADDR<br>
TEMPSENSOR_CAL2_ADDR<br>
TEMPSENSOR_CAL1_TEMP<br>
TEMPSENSOR_CAL2_TEMP<br>
TEMPSENSOR_CAL_VREFANALOG<br>
<br>
Also, the voltage regulators on these Nucleo and Discovery boards may not be outputting a full 3.3v VREF, so you may have to adjust VDDA_APPLI<br>
