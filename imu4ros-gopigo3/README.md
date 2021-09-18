# BNO055 IMU Interface and Utiliites For ROS On GoPiGo3


```
        Note that by default the axis orientation of the BNO chip looks like
        the following (taken from section 3.4, page 24 of the datasheet).  Notice
        the dot in the corner that corresponds to the dot on the BNO chip:

                           | Z axis
                           |
                           |   / X axis
                       ____|__/____
          Y axis     / *   | /    /|
          _________ /______|/    //
                   /___________ //
                  |____________|/


        NOTE: DI IMU
          - Y is direction of arrow head
          - X is toward right side when head up looking at the chip side
          - Z is coming at you when looking at the chip side

        DI IMU For ROS On GoPiGo3 (No axis remap needed if mounted like this)
          - Mount with chip side up, arrow head pointing to left side of bot
          - X is forward
          - Y is toward left side
          - Z is up
```

