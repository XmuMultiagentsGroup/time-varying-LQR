# time-varying-LQR
Start with LQR_test_mck, LQR_test_pendulum, LQR_test_swingup. These are jsut various examples to play around with. Define system dynamics there. It will also pop out the K matrix.

It will call the LQR_timevarying ftn, which in turn calls the P_dot one.

Next, the script will solve the differential equation for several cases and plots some results.

The small funcitons like Px_dot or Px_feedback are just describing system dynamics for specific examples, and I use them when I numerically solve for the output. So you'll need them if you want to run certain test scripts. You don't need them to have the LQR_timevarying function work, however.

ISSUES
-Does not work for R=0.01 (this is what Patrick's code uses). There's an inv(R) in there that makes the K-values very large. It works fine for 0.1 or above
