# GMARA
It stands for Guaranteed Multi-Agent Reach Avoid.

Temporary version of readme!!!



We implement our method on top these two tools:
1. Scots : https://gitlab.lrz.de/matthias/SCOTSv0.2
2. ALTRO : https://github.com/RoboticExplorationLab/TrajectoryOptimization.jl

we used ALTRO v0.1 which is uploaded in this repository.

# Requirements
* Julia >1.3
* c++11

## steps for execution
1. Run Altro code 
* Be careful julia directory be same as the example directory
* nom_tr.txt will be generated which is nominal controller

2. Run Makefile to compile project
* there will be two execution files
3. run abs_syn to do abstraction and synthesis
* if you faced segmentation fault or std::bad_alloc() it means there is not enough ram
4. run simulation to test the synthesized controller against model with disturbance



## steps for developement

1. Modify ALTRO (Planner):
* create *.jl file similar to examples
* change your model in dynamics! function
* select number of points and sampling time
* enter the penalty functions 

2. Modify Scots (Robustifier)
* create a *.hh in dynamics similar to examples (you dont need to change cpp files)
* enter time augmented dynamics ( one extra state variabe x_dot=1)
* select all paramets in parameters class

## directories structure

### src
This folder contains Scots library files. 
### dynamics
This folder contains Scots configurations (such as dynamics and parameters) for examples.
### TrajectoryOptimization
ALTRO Library files.
### Examples
Here there is a folder for each.  

