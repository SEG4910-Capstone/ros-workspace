## Competition Checklist
1. Update <b>*ellipse_N.yaml*</b> with the
    1. Dunwoody College coordinates
        - 44.973119 (Lat)
        - -93.290991 (Long)
    2. Primary Lever Arm (Fffset between the IMU and the antenna)
        - Based on where the GPS antenna is placed and the IMU, modify the X, Y, Z values
        - In the N, we only have one antenna so the secondary lever arm doesn't need to be accounted for
2. Update the ```magnetic_declination_radians``` in the physical_slam_localization.launch.py based on the coordinate
    - [Magnetic Declination calculator](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml)
3. Setup the IMU calibration service (https://support.sbg-systems.com/sc/kb/latest/inertial-sensors-installation/magnetic-calibration)
    - Ideally, keep the device away from source of disturbances from at least 3 meters
    - Connect the SBG ellipse with a computer and run the [sbgCenter](https://support.sbg-systems.com/sc/dev/latest/sbgcenter-application) software. From there, find the magnetometer calibration and calibrate

Finally, build the entire project, source it, then execute the project!