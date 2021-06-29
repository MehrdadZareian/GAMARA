# GAMARA
It stands for GuAranteed Multi-Agent Reach Avoid.




We implement our method on top these two tools:
1. Scots : https://gitlab.lrz.de/matthias/SCOTSv0.2
2. ALTRO : https://github.com/RoboticExplorationLab/TrajectoryOptimization.jl

we used ALTRO v0.1 which is uploaded in this repository.

# Requirements
1. Julia version : 1.3.1
* Install "Plots" library before execution.
2. c++ 11

## Steps for execution
1. Run Altro code 
* Be careful to set julia working directory same as the <example name> directory
* nom_tr.txt will be generated which is nominal controller

2. Run Makefile to compile project
* there will be two execution files
3. run abs_syn to do abstraction and synthesis
* if you faced segmentation fault or std::bad_alloc() it means there is not enough ram
4. run simulation to test the synthesized controller to see the performance of controller with perturbed model



## Steps for developement

1. Modify ALTRO (Planner):
* create *.jl file similar to examples
* change your model in dynamics! function
* select number of points and sampling time
* enter the penalty functions 

2. Modify Scots (Robustifier)
* create a *.hh similar to examples (you dont need to change cpp files)
* enter time augmented dynamics ( one extra state variabe x_dot=1)
* select all paramets in parameters class

## Directories structure

### src
This folder contains Scots library files. 
### TrajectoryOptimization
ALTRO Library files.
### Examples
Here there is a folder for each example which includes a Julia file for Altro and also example codes for ABCD.  

