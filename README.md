# Cube Satellite EPS
### Electrical Power Subsystem software implementation in Embedded C

## 1. Block Code ADC_INTERNAL 
#### Initialization of internal ADC of microcontroller register and reading from channels. 
## 2. Block Code SSP/I2C 
#### Implementation of I2C and SSP over I2C (Receive, Send, Initialization as slave and Listen).
## 3. Block Code EXTERNAL_ADC1 and EXTERNAL_ADC2 External 
#### ADC communication with Microcontroller to expand ADC channels and read more analog values through multiplexing in and out channels between internal and external ADC. 
## 4. Block Code PWM Frequency
#### Setting timers registers according to its output pin that will control boost converter in X, Y and Z lines to certain frequency by pre-scaler division concept  
## 5. Block Code PWM 
#### Setting timers registers according to its output pin that will control boost converter in X, Y and Z lines for pulse width modulation controlled by operations done core MPPT code 
## 6. Block Code MAIN CORE
#### • Initialization of Variables for default value  
#### • Loads on/off controlled SSP/I2C 
#### • MPPT code for all X, Y and Z boost lines through reading sensors data of voltage and current at inputs and outputs (Limiting current output at 2A and voltage output at 7V) 
#### • Relay kill switch control will be received on SSP/I2C 
#### • Telemetry EPS data to be sent be Sent on SSP/I2C 
#### • To implemented form as standard from libraries UART. 
