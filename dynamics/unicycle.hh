

/* state space dim */
const int state_dim=3;
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

    const char* address="/home/mehrdad/Scots+Altro/examples/unicycle/unicycle_trajectory2.txt";

    /* lower bounds of the hyper rectangle */
    state_type s_lb={{-0.5,-0.5,-0.5}};
    /* upper bounds of the hyper rectangle */
    state_type s_ub={{6,6,1.5}};
    /* grid node distance diameter */
    state_type s_eta={{0.05,0.05,0.05}};

    /* lower bounds of the hyper rectangle */
    input_type i_lb={{-1,-1}};
    /* upper bounds of the hyper rectangle */
    input_type i_ub={{1,1}};
    /* grid node distance diameter */
    input_type i_eta={{0.1,0.1}};

    /*Disturbance vector(for each state)*/
    state_type w={{0,0,0}};

    /*sampling time*/
    const double tau =0.1;

    /*number of samples in nominal trajectory (trajectory is discrete time)*/
    int trajectory_size=100;
    /* tube size is equal to number of boxes times */

    state_type epsilon_num={10,10,10};

    const int state_dim=3;
    const int input_dim=2;
} parameters;





/* we integrate the growth bound by tau sec (the result is stored in r)  */
auto radius_post = [](state_type &r, const state_type &, const input_type &u) {
    //without disturbance
    state_type w=parameters.w;
    double tau=parameters.tau;
    r[0] = r[0] + ((r[2]+w[2])*std::abs(u[0]) + std::abs(w[0])) * tau;
    r[1] = r[1] + ((r[2]+w[2])*std::abs(u[0]) + std::abs(w[1])) * tau;
    r[2] = r[2] + w[2] * tau;
};







/* we integrate the unicycle ode by tau sec (the result is stored in x)  */
auto  ode_post = [](state_type &x, const input_type &u) {
    /* the ode describing the vehicle */
    double tau=parameters.tau;

    auto rhs =[](state_type& xx,  const state_type &x, const input_type &u) { // state space model

        xx[0] = u[0]*std::cos(x[2]);
        xx[1] = u[0]*std::sin(x[2]);
        xx[2] = u[1];


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


    };
    /* simulate (use 10 intermediate steps in the ode solver) */
    scots::runge_kutta_fixed4(rhs,x,u,state_dim,tau,10);
};



