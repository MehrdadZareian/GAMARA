






const int state_dim=3;
/* input space dim */
const int input_dim=1;
/*
 * data types for the state space elements and input space
 * elements used in uniform grid and ode solvers
 */
using state_type = std::array<double,state_dim>;
using input_type = std::array<double,input_dim>;





class Parameters {
  public:


    /* lower bounds of the hyper rectangle */
    state_type s_lb={{-4,-3,-0.1}};
    /* upper bounds of the hyper rectangle */
    state_type s_ub={{10,2,4.1}};
    /* grid node distance diameter */
    state_type s_eta={{0.02,0.04,0.1}};
    /* lower bounds of the hyper rectangle */
    input_type i_lb={{-7}};
    /* upper bounds of the hyper rectangle */
    input_type i_ub={{7}};
    /* grid node distance diameter */
    input_type i_eta={{0.1}};

    /*Disturbance vector(for each state)*/
    state_type w={{0,0,0}};

    /*sampling time*/
    const double tau =0.1;

    /*number of samples in nominal trajectory (trajectory is discrete time)*/
    int trajectory_size=40;
    /* tube size is equal to number of boxes times */
    state_type epsilon_num={10,10,0};
    /*initial  point of nominal trajecotry and close loop trajecotry*/
    state_type initial_trajectory_state={{9,0,0}};

    char* address="/home/mehrdad/Scots+Altro/examples/Collision Test/cart_pole_MAA3.txt";

    const int state_dim=3;
    const int input_dim=1;


} parameters;





    /* we integrate the growth bound by tau sec (the result is stored in r)  */
    auto radius_post = [](state_type &r, const state_type &, const input_type &u) {
        //without disturbance
        state_type w=parameters.w;
        double tau=parameters.tau;
        r[0] = r[0]+0.15*r[1]+0.15*w[0]+0.15*w[1];
        r[1] = r[1]+0.15*w[1];
        r[2]=0;
    };




    /* we integrate the unicycle ode by tau sec (the result is stored in x)  */
    auto  ode_post = [](state_type &x, const input_type &u) {
        /* the ode describing the vehicle */

        double tau=parameters.tau;
        auto rhs =[](state_type& xx,  const state_type &x, const input_type &u) { // state space model

            xx[0] = x[1];
            xx[1] = u[0];
            xx[2]=1;

        };
        /* simulate (use 10 intermediate steps in the ode solver) */
        scots::runge_kutta_fixed4(rhs,x,u,state_dim,tau,10);


    };

    /*ode function with a constant distrurbance W */
    auto  ode_post_dist = [](state_type &x, const input_type &u) {
        /* the ode describing the vehicle */
        double tau=parameters.tau;
        auto rhs =[](state_type& xx,  const state_type &x, const input_type &u) { // state space model

            state_type w=parameters.w;
            xx[0] = x[1]+w[0];
            xx[1] = u[0]+w[1];
            xx[2]=1;

        };
        /* simulate (use 10 intermediate steps in the ode solver) */
        scots::runge_kutta_fixed4(rhs,x,u,state_dim,tau,10);
    };


//    class Parameters {
//      public:

//        /* lower bounds of the hyper rectangle */
//        state_type s_lb={{-4,-3,-0.1}};
//        /* upper bounds of the hyper rectangle */
//        state_type s_ub={{10,2,4.1}};
//        /* grid node distance diameter */
//        state_type s_eta={{0.02,0.04,0.1}};
//        /* lower bounds of the hyper rectangle */
//        input_type i_lb={{-7}};
//        /* upper bounds of the hyper rectangle */
//        input_type i_ub={{7}};
//        /* grid node distance diameter */
//        input_type i_eta={{0.1}};

//        /*Disturbance vector(for each state)*/
//        state_type w={{0,0,0}};

//        /*sampling time*/
//        const double tau =0.1;

//        /*number of samples in nominal trajectory (trajectory is discrete time)*/
//        int trajectory_size=40;
//        /* tube size is equal to number of boxes times */
//        state_type epsilon_num={10,10,0};
//        /*initial  point of nominal trajecotry and close loop trajecotry*/
//        state_type initial_trajectory_state={{9,0,0}};

//        char* address="/home/mehrdad/Scots+Altro/examples/Collision Test/cart_pole_MAA3.txt";

//        const int state_dim=3;
//        const int input_dim=1;


//    } parameters;






