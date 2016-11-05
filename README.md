# time-varying-LQR
Start with LQR_test_mck, LQR_test_pendulum, LQR_test_unicycle. Define system dynamics there. It will also pop out the K matrix.

It will call the LQR_timevarying ftn, which in turn calls the P_dot one.

ISSUES
-Does not work for R=0.01 (this is what Patrick's code uses). There's an inv(R) in there that makes the K-values very large. It works fine for 0.1 or above
