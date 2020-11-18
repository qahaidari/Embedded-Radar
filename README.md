# Embedded-Radar-STMicrocontroller-Cprogramming
Embedded radar sensor circuit with STM32 microcontroller and C programming for measuring target speed

# Skills learned in this project:
* GIT
* STM32CubeMx configuration code generator
* Schematic CAD software (KiCAD Eeschema): placing parts, connections in schematic, footprints, annotation, netlist
* PCB CAD software (KiCAD): loading netlist, GND pours, Vias, Gerber and Drill files for production
* Soldering components on PCB
* IDE Atollic TrueSTUDIO (Eclipse)
* C programming
* Configuring switches as external interrupts
* Communication between Interrupt Service Routine (ISR) and queue
* Configuring and using ADC
* Configuring and using timers
* Configuring LEDs
* Interfacing with LCD

# Project Description

## Introduction

RADAR stands for Radio Detection And Ranging and is a detection system that uses radio waves to determine the range, angle, or velocity of objects. It can be used to detect aircraft, ships, spacecraft, guided missiles, motor vehicles, weather formations, and terrain. Radars are now involved in various applications. It was initially used for target detection in military applications but nowadays it is used in tracking of aircrafts and vehicles and also in traffic control and space. In this project we will concentrate on motion and speed detection of objects using Radars.

The basic concept of Radar is sending an electromagnetic signal from a source which is radiating in the space. This signal will hit a moving object and then be reflected back to the source. In our case, the transmitter and receiver are on the same board of the radar device. The reflected signal from the object will be processed in the device and then finally sent to a microcontroller where the speed of the moving object is calculated using the Doppler effect.

The Doppler effect is the change of the frequency of an incident wave on a moving object in its reflected wave. The difference in frequencies is measured through a microcontroller to measure the speed of the moving object using the below formulas:

<a href="https://www.codecogs.com/eqnedit.php?latex=\large&space;\Delta&space;f=\frac{2&space;v}{\lambda}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\large&space;\Delta&space;f=\frac{2&space;v}{\lambda}" title="\large \Delta f=\frac{2 v}{\lambda}" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=\large&space;\lambda=\frac{c}{f_t}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\large&space;\lambda=\frac{c}{f_t}" title="\large \lambda=\frac{c}{f_t}" /></a>

where <a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;\large&space;\Delta&space;f" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;\large&space;\Delta&space;f" title="\large \Delta f" /></a> is the difference between transmitted and received frequencies of the signal, <a href="https://www.codecogs.com/eqnedit.php?latex=\large&space;\lambda" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\large&space;\lambda" title="\large \lambda" /></a> is the wavelength, <a href="https://www.codecogs.com/eqnedit.php?latex=\large&space;f_t" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\large&space;f_t" title="\large f_t" /></a> is the frequency of the transmitted signal, <a href="https://www.codecogs.com/eqnedit.php?latex=\large&space;c" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\large&space;c" title="\large c" /></a> is the speed of light and <a href="https://www.codecogs.com/eqnedit.php?latex=\large&space;v" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\large&space;v" title="\large v" /></a> is the velocity of the object.
