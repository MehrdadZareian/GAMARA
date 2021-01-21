#ifndef DETAILS_H
#define DETAILS_H

#include<vector>
#include "scots.hh"

/* state space dim */
const int state_dim=6;
/* input space dim */
const int input_dim=2;
using state_type = std::array<double,state_dim>;
using input_type = std::array<double,input_dim>;

class Details
{
public:
    Details();

     Details& operator = (const Details other);
    /* sampling time */
   // double tau = 0.05;

    state_type s_lb;
    state_type s_ub;
    state_type s_eta;
    input_type i_lb={{-1}};
    /* upper bounds of the hyper rectangle */
    input_type i_ub={{ 1}};
    /* grid node distance diameter */
    input_type i_eta={{1}};
    std::vector<state_type> trajectory_states;
    std::vector<state_type> syn_trajectory_states;
    std::vector<state_type> syn_trajectory_states2;

    double epsilon;


};

#endif // DETAILS_H
