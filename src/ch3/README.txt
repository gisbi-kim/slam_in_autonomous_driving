# Contents 
1. `process_gnss`: sparse (10hz) global prior observation 
    - After executing it, you can fine the result file at `data/ch3/gnss_output.txt`
2. `run_imu_integration`: imu only propagtations with no any measurements 
    - It would diverge so fast. 
3. `./run_eskf_gins --with_odom=false`: SE3-like loss from GNSS only 
    - It would generate a reasonable continous (100hz) poses more denser and smoother than the raw GNSS measurement.
    - However, you can see some diverging moments when GNSS signals are off. 
4. `./run_eskf_gins --with_odom=true`: SE3-like loss + Wheel-based Velocity loss 
    - This shows the best performance. It also would fix the failure points of the above experiment 3.