#include <fstream>
#include <iostream>
#include <array>
#include <vector>
#include <set>
#include <algorithm>
#include <map>
#include <unordered_map>


/* SCOTS header */
#include "scots.hh"
/* ode solver */
#include "RungeKutta4.hh"
/* time profiling */
#include "TicToc.hh"
/* memory profiling */
#include <ctime>
#include <sys/resource.h>
#include "merge.hh"


#define pi 3.14

/*Here we test the synthesized controller with forward simulation*/


/*this function checks whether a points is inside a box represented by upper right and lower left point*/
bool check_inside(state_type left,state_type right, state_type point ){
    for (int i = 0; i < state_dim; ++i) {
        if(point[i] < left[i] || point[i] > right[i])
            return false;
    }
    return true;
}



int main(){
    state_type initial_trajectory_state=parameters.initial_trajectory_states[parameters.trajectory_num];
    std::vector<state_type> nominal_trajectory_states=trajectory_simulation(parameters);

    int trajectory_size=parameters.trajectory_size;

    /*Target set lower left and upper right*/
    state_type target_left;
    state_type target_right;

    /*specifying target set with lower left and upper right point (last point of trajectory +- radius */
    /* last dimention is time which has radius zero (for sake of implementation we considered a small value)*/
    for (int i = 0; i < state_dim; ++i) {
        double target_radius;
        if(i!=state_dim-1)
            target_radius=parameters.epsilon_num[i]*parameters.s_eta[i];
        else
            target_radius=parameters.s_eta[i]/1e10;
        target_left[i]=nominal_trajectory_states[trajectory_size-1][i]-target_radius;
        target_right[i]=nominal_trajectory_states[trajectory_size-1][i]+target_radius;
    }

    /*uniform grid for state space*/
    scots::UniformGrid ss(state_dim,parameters.s_lb,parameters.s_ub,parameters.s_eta);
    /*uniform grid for input space*/
    scots::UniformGrid is(input_dim,parameters.i_lb,parameters.i_ub,parameters.i_eta);


    /*size of Global state space uniform grid */
    long long unsigned int N=ss.size();
    /*this array stores whether a point is inside of tube or not*/
    bool* inside_of_area= new bool[N];
    /*mapping from global index to local index*/
    unsigned int* GtoL=new unsigned int[N];
    /*mapping from local index to global index*/
    std::vector<abs_type>  LtoG;
    scots::Abstraction<state_type,input_type,Parameters> abs(ss,is);

    abs.map_initialization(nominal_trajectory_states,ss,parameters,target_left,target_right,inside_of_area,LtoG,GtoL);

    scots::StaticController con;
    if(!read_from_file(con,"controller")) {
        std::cout << "Could not read controller from controller.scs\n";
        return 0;
    }



    std::cout << "\nSimulation:\n " << std::endl;

    /* This part is used to see how many points of trajectory included in control domain
  for(int i=0;i<trajectory_size;i++)
       std::cout<<"check "<<i<<"- "<<con.check<state_type,input_type>(nominal_trajectory_states[i],GtoL,inside_of_area)<<std::endl;
  */


    state_type x=initial_trajectory_state;
    std::vector<state_type> syn_trajectory_states;
    syn_trajectory_states.push_back(x);



    auto target_state = [&ss,&target_left,&target_right](const state_type& x) {

        if(check_inside(target_left,target_right,x))
            return true;

        return false;
    };
    /*checking whether states is inside control domain or not*/
    if(!con.check<state_type,input_type>(x,GtoL,inside_of_area))
        std::cout<<"This state is not inside of control domain"<<std::endl;

    else{
        std::cout<<"Closed loop from an specified initial point and with disturbance"<<std::endl;
        while(1) {
            std::vector<input_type> u = con.get_controller<state_type,input_type>(x,GtoL,inside_of_area);
            std::cout << x[0] <<  " "  << x[1] <<"\n";
            ode_post_dist (x,u[0]);

            syn_trajectory_states.push_back(x);

            if(target_state(x) ) {
                std::cout << "Arrived: " << x[0] <<  " "  << x[1] << std::endl;
                break;
            }
        }
    }




    return 1;

}
