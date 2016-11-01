# time-varying-LQR
Start with LQR_test script. Define system dynamics there. It will also pop out the K matrix.

It will call the LQR_timevarying ftn, which in turn calls the P_dot one.

Right now the example has a constant A. However, if A varies with time, it would be easy to modify LQR_test to store A at each time step in a different cell. The LQR_timevarying function will not have to change.

ISSUES
-Does not work for R=0.01 (this is what Patrick's code uses). There's an inv(R) in there that makes the K-values very large. It works fine for 0.1
