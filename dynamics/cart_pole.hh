

/* state space dim */
const int state_dim=5;
/* input space dim */
const int input_dim=1;
/*
 * data types for the state space elements and input space
 * elements used in uniform grid and ode solvers
 */
using state_type = std::array<double,state_dim>;
using input_type = std::array<double,input_dim>;

using abs_type = scots::abs_type;

double L_first[71][16];
double L_second[71][16];


/*Assigning values to parametrs*/
class Parameters {
  public:

    const char* address="/home/mzareian/Scots+Altro/examples/cart_pole_MA/cart_pole_MAA3.txt";
    const char* address_gb="/home/mzareian/Scots+Altro/examples/cart_pole_MA/GB9.txt";

    /* lower bounds of the hyper rectangle */
    state_type s_lb={{-0.2,-2,1.2,-6.3,-0.1}};
    /* upper bounds of the hyper rectangle */
    state_type s_ub={{5.5,4.4,4.7,5,4.1}};
    /* grid node distance diameter */
    state_type s_eta={{0.02,0.045,0.02,0.08,0.1}};

    /* lower bounds of the hyper rectangle */
    input_type i_lb={-7};
    /* upper bounds of the hyper rectangle */
    input_type i_ub={7};
    /* grid node distance diameter */
    input_type i_eta={{0.2}};


    /*Disturbance vector(for each state)*/
    state_type w={{0,0.03,0,0,0}};

    /*sampling time*/
    const double tau =0.1;

    /*number of samples in nominal trajectory (trajectory is discrete time)*/
    int trajectory_size=100;
    /* tube size is equal to number of boxes times */

    state_type epsilon_num={8,10,11,11,0};

    const int state_dim=state_dim;
    const int input_dim=input_dim;
} parameters;





/* we integrate the growth bound by tau sec (the result is stored in r)  */
auto radius_post = [](state_type &r, const state_type &, const input_type &u) {
   state_type r_temp;
   state_type w=parameters.w;
    for (int i=0;i<4;i++)
        r_temp[i]=0;

    int index=(u[0]+7)/0.2;
    for (int i=0;i<4;i++)
        for(int j=0;j<4;j++){
            r_temp[i]+= r[j]*L_first[index][4*j+i] + w[j]*L_second[index][4*j+i];
        }

    for (int i=0;i<4;i++)
        r[i]=r_temp[i];

    r[4]=0;
};







/* we integrate the unicycle ode by tau sec (the result is stored in x)  */
auto  ode_post = [](state_type &x, const input_type &u) {
    /* the ode describing the vehicle */
    double tau=parameters.tau;
    auto rhs =[](state_type& xx,  const state_type &x, const input_type &u) { // state space model


        double g = 9.8;
        double M_c=1.0;
        double M_p=0.1;
        double M_t=M_p+M_c;
        double l=0.5;
        double c1 = u[0]/M_t;
        double c2 = M_p*l/M_t;
        double c3=l*4.0/3.0;
        double c4=l*M_p/M_t;

        double F=(g*std::sin(x[2])-std::cos(x[2])*(c1+c2*x[3]*x[3]*std::sin(x[2])))/(c3-c4*std::cos(x[2])*std::cos(x[2]));
        double G= (c1+c2*x[3]*x[3]*std::sin(x[2])) - c4* std::cos(x[2])*F;

        xx[0] =x[1];
        xx[1] =G;
        xx[2] =x[3];
        xx[3] =F;
        xx[4]=1;

    };
    /* simulate (use 10 intermediate steps in the ode solver) */
    scots::runge_kutta_fixed4(rhs,x,u,state_dim,tau,10);


};



auto  ode_post_dist = [](state_type &x, const input_type &u) {
    /* the ode describing the vehicle */
    double tau=parameters.tau;
    auto rhs =[](state_type& xx,  const state_type &x, const input_type &u) { // state space model
        state_type w=parameters.w;
        double g = 9.8;
        double M_c=1.0;
        double M_p=0.1;
        double M_t=M_p+M_c;
        double l=0.5;
        double c1 = u[0]/M_t;
        double c2 = M_p*l/M_t;
        double c3=l*4.0/3.0;
        double c4=l*M_p/M_t;

        double F=(g*std::sin(x[2])-std::cos(x[2])*(c1+c2*x[3]*x[3]*std::sin(x[2])))/(c3-c4*std::cos(x[2])*std::cos(x[2]));
        double G= (c1+c2*x[3]*x[3]*std::sin(x[2])) - c4* std::cos(x[2])*F;

        xx[0] =x[1]+w[0];
        xx[1] =G+w[1];
        xx[2] =x[3]+w[2];
        xx[3] =F+w[3];
        xx[4]=1;
    //    std::cout<<w[0]<<" "<<w[1]<<" "<<w[2]<<" "<<w[3]<<std::endl;

    };
    /* simulate (use 10 intermediate steps in the ode solver) */
    scots::runge_kutta_fixed4(rhs,x,u,state_dim,tau,10);


};





