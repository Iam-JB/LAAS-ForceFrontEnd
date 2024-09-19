# LAAS-ForceFrontEnd

Electronics development to acquire and process force measurements produced by an actuator in order to achieve torque control.

Here you will find the technical review : ![Technical Review](./TechnicalReview.pdf)

Files sumarry :
   ![ADS1235EVM.kicad_sch](./ADS1235EVM.kicad_sch)   KiCAD Schematic of our own acquisition PCB
   ![ADS1235EVM.kicad_pcb](./ADS1235EVM.kicad_pcb)   KiCAD PCB Routing of our own acquisition PCB
   ![ForceDetection.ino](./ForceDetection.ino)       Code in order to flash Raspberry Pi Pico
   ![ExtractData.py](./ExtractData.py)               Parceur that extracts force value from an external file ('xxx.bin') and plots through MatPlotLib
   ![RealTimeParceur.py](./RealTimeParceur.py)       Parceur that extracts data directly from serial port and plots through PlotJuggler
