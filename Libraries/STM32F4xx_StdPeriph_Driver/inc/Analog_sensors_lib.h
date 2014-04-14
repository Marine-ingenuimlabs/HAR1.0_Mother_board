#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)
void AnalogSensors_Config(uint32_t *ADC3ConvertedValue);
void AnalogSensors_Update( uint32_t *ADC3ConvertedValue, uint32_t *ADC3ConvertedVoltage);