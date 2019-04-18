# Note
Code developed originally by Dr. Alex Liniger et al.

# MPCC
Simulation environment of the Model Predictive Contouring Controller (MPCC) for Autonomous Racing developed by the Automatic Control Lab (IfA) at ETH Zurich


## How to run

### Before running code
1) Install [hpipm](https://github.com/giaf/hpipm) including the python interface
2) alternativly install Yalmip or CVX
### Run code
0) in simulation.m change to the optimization framework you use (hpipm, Yalmip, CVX)
1) run simulation.m
2) play with the tunning in getMPC_vars.m
3) change the car model between FullSize and ORCA
4) change the track layout between the ORCA and the RCP track

## Example
<img src="https://github.com/alexliniger/MPCC/blob/master/Images/MPC_sim.gif" width="700" />
