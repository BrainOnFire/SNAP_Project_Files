# S.N.A.P. Version 6.0

*This README is a work in progress*

<img src="SNAP Project PCB Files/PCB Files V6.0/Images/PCB Front 2.jpg" width="300px"></a>

Soil Nutrient Analisys Prototype or S.N.A.P. for short, is an agricultural device that allows farmers to know the best type of nutrient present in a soil sample. The most common types of nutrients found in the ground are Nitrogen, Phosphorus, and Potassium. Photo-sensing sensors can observe these nutrients when an infrared bream hits a sample. Having this information in less than 10 minutes allows the farmer to decide which crop to plant in the selected soil, saving a lot of time and money. The device has come a long way, as this is the sixth iteration of the project. SNAP 6.0 is a BLE, Low-Power, feature rich sensing device. Here are some specs of the device:

- STM32WB35CCU7A as main MCU (Ultra-low-power dual core ARM Coretx-M4 with FPU, BLuetooth 5.2, 256KB Flash, 32MHz)
- Integrated Lithium-Ion battery charging with MCP73831 (Charge current 500mA, 4.2V Regulation, Reverse Discharge Protection)
- Battery protection using AP9101C (overcharge voltage, over-discharge voltage, overcharge current, over-discharge current)
- Boost Converter from 3.0V to 5.2V using MT3608 (turns on/off by MCU)
- Temperature and humidity sensing using SHT31-DIS
- OLED Screen for displaying information with low consumption
- PCB Antenna with impedance matching for Bluetooth Low Energy
- USB Type-C 2.0 for 5V power supply and communication

## PCB Design
The design and development of the printed circuit board were done using KiCAD 6.0 . The PCB in question is a 4-layer board with a thickness of 1.62 mm and overall dimensions of 66.80 mm x 47.34 mm. The layer stack up is Signal - Ground - Ground - Mixed . 

A USB-C Type Receptacle was used for power supply coupled with a USBLC6-2SC6 ESD protection chip. Serial Wire is used for the programming of the STM32 chip. A MOSFET and a Schottky Diode is used in order to protect the device when both battery and USB are connected and supplying power to the system. Both inputs are regulated by the ADP121 LDO which outputs a voltage of 3.0V to the MCU and to the boost-up converter. The amplification system, the three IR LEDs, the Phototransistor, the SHT31 and OLED screen are supplied by the 5.2V outputted by the boost-up converter MT3608. Three 2N700 MOSFETs are used to control the IR LEDs through the MCU. 

A total of 27 decoupling capacitors of different values and sizes are used to filter unwanted frequencies. 

A filter PI and RF Band Pass filter are used for the 2.4GHz PCB BLE antenna. The values of capacitance and inductance were calculated using the impedance of the PCB antenna (obtained from [AN5129](https://www.st.com/resource/en/application_note/an5129-low-cost-pcb-antenna-for-24ghz-radio-meander-design-for-stm32wb-series-stmicroelectronics.pdf)), in order to obtain a matching impedance of 50 ohm. The next two images show the first and second page of the schematic; every part of the device is clearly named.

<img src="SNAP Project PCB Files/PCB Files V6.0/Images/Page 1 KiCAD Schematic.jpg" width="300px"></a>
<img src="SNAP Project PCB Files/PCB Files V6.0/Images/Page 2 KiCAD Schematic.jpg" width="300px"></a>

This PCB was manufactured by JLCPCB using the controlled impedance stackup JLC04161H-7628 for a dielectric constant of 4.6 . This provides a matching impedance of 50 ohm for the traces of the embedded antenna that mitigate potential power losses due to parasitic capacitances. The SMD components were soldered using a SMD reflow process with a heat gun.

<img src="SNAP Project PCB Files/PCB Files V6.0/Images/PCB Design.jpg" width="310px"></a>
<img src="SNAP Project PCB Files/PCB Files V6.0/Images/3D PCB Front.jpg" width="300px"></a>

## Firmware and hardware integration
