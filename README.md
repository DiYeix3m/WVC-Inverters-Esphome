# WVC-Inverters-Esphome
WVC RF microinverters components for esphome
based on the work by [@krywenko ](https://github.com/krywenko/WVC-Inverter-R2-R3-HC-12--and--Modem)

It works with both HW revisions of WVC inverters that work with HC-12 RF module

R2 inverters supported:
- WVC295
- WVC300
- WVC350
- WVC600
- WVC860
- WVC1200

R3 inverters supported:
- All models

Only 4 wires needed, VCC (3.3v), GND, TX-> RX and RX->TX

Currently, works with only one inverter at the time, however you can create two or more components and they will answer randomly asigning values to the first component

![imagen](https://github.com/user-attachments/assets/91e09528-073c-4d01-b305-02d6cf64a7ee)
