# Note
Code developed originally by Dr. Alex Liniger et al. My version uses an alternative to Quadprog Quadratic Problem optimization solver. This method was coded from scratch.

# MPCC
Simulation environment of the Model Predictive Contouring Controller (MPCC) for Autonomous Racing developed by the Automatic Control Lab (IfA) at ETH Zurich


## How to run

### Before running code
1) Install ...
2) Use quadprog, "MPC_vars.interface = 'quadprog';" Note: use of Quadprog yeilds more accurate solution than my QP solver.
### Run code
1) run simulation.m
2) play with the tunning in getMPC_vars.m
3) change the car model between FullSize and ORCA
4) change the track layout between the ORCA and the RCP track

## Example
<img src="https://github.com/alexliniger/MPCC/blob/master/Images/MPC_sim.gif" width="700" />
