# time-varying-LQR
Start with LQR_test script. Define system dynamics there. It will also pop out the K matrix.

It will call the LQR_timevarying ftn, which in turn calls the P_dot one.

Right now the example has a constant A. However, if A varies with time, it would be easy to modify LQR_test to store A at each time step in a different cell. The LQR_timevarying function will not have to change.

ISSUES
-Something wrong with using small values of R(t). Need to investigate further.
-I haven't quite figured out what to do with K(t) once I find it. There are some plots I was playing around with but they are not what I'm looking for.
