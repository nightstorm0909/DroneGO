# DroneGO (Drone Guarding Object)

## Introduction

This was the final project for one of my courses. The project uses another project [cvdrone](https://github.com/puku0x/cvdrone) as a base for communicating with 
*Parrot AR Drone 2.0*.

The main aim of the project was to make the drone guard an object. The drone has 2 modes:

	1. *Safe mode*: In this mode, the drone will be idle sitting opposite to the object with the object in focus.

	2. *Tracking mode*: This mode is triggered whenever any intruder picks up the object and tries to run away 
			    with it. In this mode, the drone will start to follow the object while sending a text message 
			    to the owner through facebook. While tracking, the owner can ask the drone about its status
			    through facebook, which will make the drone respond by send the picture of what it is seeing
			    as well as with the status of its flight. The drone also send a stress signal to any laptop
			    connected to the wifi hotspot to use their speaker to raise an alarm.

## Requirements

Python modules used: fbchat, pygame
OS used: Ubuntu 16.04

## Usage:

To build the code, go to DroneGO/build/unix and use the makefile file to
build the system.

In order to connect to the internet during the flight, you need to 
conect the laptop to different wifi hotspots: one created by the 
drone for communication and the other hotpsot which is connected to 
the internet.

For reference, check the picture:

[networking](https://github.com/nightstorm0909/DroneGO/blob/master/images/networking.png)