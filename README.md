# RIT-Racing Core

## Overview
Most of our projects use the same features of the STM32s. They require internal/external clocks,
GPIO functionality, communication protocols like CAN, SPI, or UART, the ability to read analog inputs, etc.
For STM32s and most other microcontrollers, there is a lot of overhead that is needed to include these features
in a project. Not only is it a lot of work to write this code over and over again, but it is a slow process that leaves
lots of room for error.

The Core is our solution to that problem.

## What is the Core?
The Core is library of drivers, built by us, that provides an easy way to interface with hardware on an STM32 microcontroller.
It consists of commonly-used functions, spanning many hardware abilities, that provide simple ways to do common tasks.

You can think of it as a way to "Arduino-ify" the STM32s we use.

## How is the Core used?
The Core is implemented as a GitHub Submodule, which is a way of including a GitHub Repository inside of another project,
allowing it to be used.

Although the Core is very generic and can be used by many different projects, there is still a way to customize it
to your use: the **core_config.h** file. Here, you will find many definitions relating to different parts of the Core code.
This file allows you to change variables such as clock speeds, pin functionality, specific peripheral parameters, and more.  
The **core_config.h** file you will be using in your projects, however, is not the one in the Core. Every new project needs its 
own unique config file to work. This has been included for you in the [template project](https://github.com/RITRacingSoftware/STM32G4xx-Template.git).

**bold** *italics*
# BIG
## Medium

[//]: # ()
[//]: # (## Why do we need the Core?)

[//]: # (The Core is a helpful for a few reasons:)

[//]: # ()
[//]: # ()
[//]: # (### 1. Standardization)

[//]: # ()
[//]: # (As all projects will need access to the same functionality, it is most efficient to just write that code once in a generic way tha)

[//]: # ()
[//]: # (### 2. Simplicity)

[//]: # ()
[//]: # (### 3. Adaptability)

[//]: # (Say you want to rework a driver for the way a driver works, say to make it more efficient, or to add a feature to it.)

[//]: # (With the Core, that change only has to be made once.)

[//]: # (Once the modification to the Core has been pushed to GitHub, it's open for any project to use it seamlessly.)

[//]: # (All that has to be done is for the Core submodule inside a project to be updated, and the new functionality will work.)

[//]: # ()
[//]: # (However, imagine that process without the Core. Not only would you have to manually make that change in every)

[//]: # (project that uses that functionality, but as every project would implement it slightly differently, multiple different)

[//]: # (versions of the change would have to be made to work with each project.)

[//]: # ()
[//]: # (The code inside the Core shouldn't be changed when you're using it in your projects, if it needs to be changed it should be done so)

[//]: # (by a qualified member inside the core)