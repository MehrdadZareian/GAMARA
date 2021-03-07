
#include <vector>
#define verbose true
#define pi 3.14


/* state space dim */
const int state_dim=4;
/* input space dim */
const int input_dim=2;
/*
 * data types for the state space elements and input space
 * elements used in uniform grid and ode solvers
 */
using state_type = std::array<double,state_dim>;
using input_type = std::array<double,input_dim>;

using abs_type = scots::abs_type;


/*Assigning values to parametrs*/
class Parameters {
  public:

    const char* address="nom_tr.txt";

    /* lower bounds of the hyper rectangle */
state_type s_lb={{-2,-2,-0.6,0}};
    /* upper bounds of the hyper rectangle */
state_type s_ub={{17,17,1.7,10.9}};
    /* grid node distance diameter */
state_type s_eta={{0.03,0.03,0.03,0.1}};

    /* lower bounds of the hyper rectangle */
input_type i_lb={{-1,-3}};
    /* upper bounds of the hyper rectangle */
input_type i_ub={{5,3}};
    /* grid node distance diameter */
input_type i_eta={{0.3,0.3}};

    /*Disturbance vector(for each state)*/
state_type w={{0,-0.03,-0.03}};

    /*sampling time*/
    const double tau =0.1;

    /*number of samples in nominal trajectory (trajectory is discrete time)*/
    int trajectory_size=110;
    /* tube size is equal to number of boxes times */

    state_type epsilon_num={7,7,7,0};

    const int state_dim=4;
    const int input_dim=2;
    
    const int agent_num=5;
    const int trajectory_num=4; //starts from 0
    std::vector<state_type> initial_trajectory_states{ {{0,0,pi/4,0}},   //1
                                                       {{0,1.5,pi/4,0}}, //2
                                                       {{1.5,0,pi/4,0}}, //3
                                                       {{0,-1.5,pi/4,0}},//4
                                                       {{1.5,0,pi/4,0}}  //5
                                                     };

} parameters;





/* we integrate the growth bound by tau sec (the result is stored in r)  */
inline auto radius_post = [](state_type &r, const state_type &, const input_type &u) {
    //without disturbance
    static state_type w=parameters.w;
    static double tau=parameters.tau;
    r[0] = r[0] + ((r[2]+std::abs(w[2]))*std::abs(u[0]) + std::abs(w[0])) * tau;
    r[1] = r[1] + ((r[2]+std::abs(w[2]))*std::abs(u[0]) + std::abs(w[1])) * tau;
    r[2] = r[2] + std::abs(w[2]) * tau;
    r[3]=0;
};







/* we integrate the unicycle ode by tau sec (the result is stored in x)  */
inline auto  ode_post = [](state_type &x, const input_type &u) {
    /* the ode describing the vehicle */
    double tau=parameters.tau;

    auto rhs =[](state_type& xx,  const state_type &x, const input_type &u) { // state space model

        xx[0] = u[0]*std::cos(x[2]);
        xx[1] = u[0]*std::sin(x[2]);
        xx[2] = u[1];
        xx[3] = 1;

    };
    /* simulate (use 10 intermediate steps in the ode solver) */
    scots::runge_kutta_fixed4(rhs,x,u,state_dim,tau,10);

};


auto  ode_post_dist = [](state_type &x, const input_type &u) {
    /* the ode describing the vehicle */
    double tau=parameters.tau;
    auto rhs =[](state_type& xx,  const state_type &x, const input_type &u) { // state space model
        state_type w=parameters.w;
        xx[0] = u[0]*std::cos(x[2])+w[0];
        xx[1] = u[0]*std::sin(x[2])+w[1];
        xx[2] = u[1]+w[2];
        xx[3] = 1;

    };
    /* simulate (use 10 intermediate steps in the ode solver) */
    scots::runge_kutta_fixed4(rhs,x,u,state_dim,tau,10);
};

std::vector<state_type> trajectory_simulation(Parameters& parameters){
    std::vector<state_type> nominal_trajectory_states;

    int trajectory_size=parameters.trajectory_size;
    int tr_num=parameters.trajectory_num;
    state_type initial_trajectory_state=parameters.initial_trajectory_states[tr_num];
    std::ifstream inFile;
    inFile.open(parameters.address);


    std::vector<input_type> u_temp(parameters.agent_num);
    std::vector<input_type> inputs(trajectory_size);

    for ( int i = 0; i <trajectory_size ; i++) {
        for (int k = 0; k < parameters.agent_num ; ++k) {
            for(int q=0;q<input_dim;q++)
                inFile >> u_temp[k][q];
        }
      inputs[i]=u_temp[tr_num];
    }

    state_type temp_state;
    nominal_trajectory_states.push_back(initial_trajectory_state);
    temp_state=initial_trajectory_state;
    state_type min_x=initial_trajectory_state;
    state_type max_x=initial_trajectory_state;


    for ( int i = 0; i <trajectory_size-1 ; i++)
    {
        ode_post (temp_state,inputs[i]);
        for (int i = 0; i < state_dim; ++i) {
            if(min_x[i]>temp_state[i])
                min_x[i]=temp_state[i];
            if(max_x[i]<temp_state[i])
                max_x[i]=temp_state[i];
        }

        nominal_trajectory_states.push_back(temp_state);
        if(verbose)
            std::cout<< i+2<<"- " << nominal_trajectory_states[i+1][0]<< "  " <<nominal_trajectory_states[i+1][1]<<"  " <<nominal_trajectory_states[i+1][2]<<"  " << std::endl;
    }
    if(verbose)
        for (int i = 0; i < state_dim; ++i)
            std::cout<<i<<" min:"<<min_x[i]<<" max:"<<max_x[i]<<std::endl;

    return nominal_trajectory_states;

}








