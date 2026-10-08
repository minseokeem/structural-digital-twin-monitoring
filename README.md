# Structural Digital Twin Monitoring System

## Overview
This project develops a real-time structural monitoring system based on digital twin technology. It integrates 3D structural modeling, AI-assisted sensor placement, wireless sensing, and state estimation to monitor structural displacement and visualize condition changes in a digital twin environment.

![Project Poster](media/project_poster_github.png)

## My Role
- Designed the sensor module hardware and PCB
- Integrated ESP32-C3, DW3000 UWB, and ADXL345 accelerometer
- Improved the UWB-based localization algorithm
- Performed experiments and analyzed ranging/localization data

## Key Improvements
- Replaced incremental displacement accumulation with absolute position re-estimation
- Added initial range-bias calibration and outlier rejection
- Reduced stationary position-estimate fluctuation to approximately ±10 mm
- Added motion-triggered UWB revalidation using accelerometer events

## System Components
- ESP32-C3
- DW3000 UWB
- ADXL345 accelerometer
- UWB anchors and tags
- Web-based digital twin interface
