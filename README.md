# DroneGO (Drone Guarding Object)

## Introduction

This was the final project for one of my courses. The project uses another project "cvdrone" as a base for communicating with 
*Parrot AR Drone 2.0*.

The main aim of the project was to make the drone guard an object. The drone has 2 modes:
	1. *Safe mode*: In this mode, the drone will be idle sitting opposite to the object with the object in focus.
	2. *Tracking mode*: This mode is triggered whenever any intruder picks up the object and tries to run away with it. In this mode,
					the drone will start to follow the object while sending a text message to the owner through facebook. While tracking,
					the owner can ask the drone about its status through facebook, which will make the drone respond by send the
					picture of what it is seeing as well as with the status of its flight.

## Requirements

Python modules used: fbchat, pygame

## Usage:

