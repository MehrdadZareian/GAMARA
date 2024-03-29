

/** @file **/
#ifndef ABSTRACTION_HH_
#define ABSTRACTION_HH_

#include <iostream>
#include <cstring>
#include <memory>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <array>

#include "UniformGrid.hh"
#include "TransitionFunction.hh"

/* size of loop iteration block allocated to each worker at a time */
#define CHUNK_SIZE 800


/** @namespace scots **/ 
namespace scots {

/** @cond **/
/* default parameter for the third parameter of Abstraction::compute_gb  */
namespace params {
auto avoid_abs = [](const abs_type&) noexcept {return false;};
}
using abs_type = scots::abs_type;
/** @endcond **/


/**
 * @class Abstraction
 *
 * @brief Computation of the transition function of symbolic model/abstraction
 *
 * It implements in compute_gb the computation of the transition function based
 * on a growth bound. Additionally, it provides the functions print_post and
 * print_post_gb to print the center of the cells that cover the attainable set
 * which is computed from the growth bound.
 *
 * See
 * - the manual in <a href="./../../manual/manual.pdf">manual.pdf</a>
 * - http://arxiv.org/abs/1503.03715 for theoretical background
 *
 **/
template<class state_type, class input_type,class Parameters>
class Abstraction {
private:
    /* grid information of state alphabet */
    const UniformGrid m_state_alphabet;
    /* grid information of input alphabet */
    const UniformGrid m_input_alphabet;
    /* measurement error bound */
    std::unique_ptr<double[]> m_z;
    /* print progress to the console (default m_verbose=true) */
    bool m_verbose=true;
    /* to display the progress of the computation of the abstraction */
    void progress(const abs_type& i, const abs_type& N, abs_type& counter) {
        if(!m_verbose)
            return;
        if(((double)i/(double)N*100)>=counter){
            if((counter%10)==0)
                std::cout << counter;
            else if((counter%2)==0)
                std::cout << ".";
            counter++;
        }
        std::flush(std::cout);
        if(i==(N-1))
            std::cout << "100\n";
    }
public:
    /* @cond  EXCLUDE from doxygen*/
    /* deactivate standard constructor */
    Abstraction() = delete;
    /* cannot be copied or moved */
    Abstraction(Abstraction&&) = delete;
    Abstraction(const Abstraction&) = delete;
    Abstraction& operator=(Abstraction&&)=delete;
    Abstraction& operator=(const Abstraction&)=delete;
    /* @endcond */

    /** @brief constructor with the abstract state alphabet and abstract input
   * alphabet (measurement error bound is set to zero)
   *
   *  @param state_alphabet - UniformGrid representing the abstract state alphabet
   *  @param input_alphabet - UniformGrid representing the abstract input alphabet
   **/
    Abstraction(const UniformGrid& state_alphabet,
                const UniformGrid& input_alphabet) :
        m_state_alphabet(state_alphabet),
        m_input_alphabet(input_alphabet),
        m_z(new double[state_alphabet.get_dim()]()) {
        /* default value of the measurement error
     * (heurisitc to prevent rounding errors)*/
        for(int i=0; i<m_state_alphabet.get_dim(); i++)
            m_z[i]=m_state_alphabet.get_eta()[i]/1e10;
    }

    /**
   * @brief computes the transition function
   *
   * @param[out] transition_function - the result of the computation
   *
   * @param[in] system_post - lambda expression of the form
   *                          \verbatim [] (state_type &x, const input_type &u) ->  void  \endverbatim
   *                          system_post(x,u) provides a numerical approximation of ODE
   *                          solution at time tau with initial state x and input u \n
   *                          the result is stored in x
   *
   * @param[in] radius_post - lambda expression of the form
   *                          \verbatim [] (state_type &r, const state_type& x, const input_type &u) -> void  \endverbatim
   *                          radius_post(x,u) provides a numerical approximation of
   *                          the growth bound for the cell (with center x, radius  r) and input u\n
   *                          the result is stored in r
   *
   * @param[in] avoid  - OPTIONALLY provide lambda expression of the form
   *                     \verbatim [] (const abs_type &i) -> bool \endverbatim
   *                     returns true if the abstract state i is in the avoid
   *                     set; otherwise returns false
   *
   * The computation proceeds in two loops. In the first loop the cornerIDs are
   * comuted, which represent the cell IDs that cover the over-approximation of the attainable set:
   * \verbatim corner_IDs[i*M+j+0] \endverbatim = lower-left cell ID of the
   * integer hyper-interval of cell IDs that cover the over-approximation of the
   * attainable set of cell with ID=i under input ID=j
   * \verbatim corner_IDs[i*M+j+1] \endverbatim = upper-right  of the
   * integer hyper-interval of cell IDs that cover the over-approximation of the
   * attainable set of cell with ID=i under input ID=j
   *
   * In the second loop the data members of the TransitionFunction are computed.
   *
   **/




    /*we need mapping between indexes in global frame and local frame (indexes of points inside tube) for doing local abstraction */
    /*This function is for calculating these mappings */
    void map_index_calculation(std::vector<state_type>& trajectory_states,scots::UniformGrid ss, Parameters parameters
                            ,state_type target_left,state_type target_right
                            ,bool* inside_of_area
                            ,std::vector<abs_type>& LtoG
                            ,unsigned int* GtoL){

        int dim=parameters.state_dim;
        abs_type index=0;
        std::vector<abs_type> lb(dim);  /* lower-left corner location of each box */
        std::vector<abs_type> ub(dim);  /* upper-right corner location of each box */
        std::vector<abs_type> no(dim);  /* number of cells per dim inside each box */
        std::vector<abs_type> cc(dim);  /* coordinate of current cell in the post in current box */


        state_type r;/* radius of each box */
        state_type eta=parameters.s_eta; /* size of each cell in state space (descritization parameter) */
        state_type m_z; /*a very small value for avoiding aligning radius with cells*/

        /*number of cells as a radius of box*/
        state_type epsilon_num = parameters.epsilon_num;

        int Tn=parameters.trajectory_size;
        /* for out of bounds check */
        state_type lower_left;
        state_type upper_right;
        /* fill in data */
        for(int i=0; i<dim; i++) {
            // r[i]=epsilon[i];
            r[i]=epsilon_num[i]*eta[i];
            lower_left[i]=ss.get_lower_left()[i];
            upper_right[i]=ss.get_upper_right()[i];
            m_z[i]=eta[i]/1e10;
        }
        /* determine post */



        /*In this loop we are iterating over all points of tube in order to fill global to local and local to global mapping arrays */
        for (int i = 0; i < Tn; ++i) {
            abs_type npost=1;
            state_type Left,Right;
            for(int k=0; k<dim; k++) {
                /* check for out of bounds */
                double left = trajectory_states[i][k]-r[k]-m_z[k];
                double right = trajectory_states[i][k]+r[k]+m_z[k];
                if(i==Tn){
                    left = target_left[i];
                    right = target_right[i];
                }
                Left[k]=left;
                Right[k]=right;
                if(left <= lower_left[k]-eta[k]/2.0  || right >= upper_right[k]+eta[k]/2.0) {
                    std::cout<<"uniform grid is too small"<<std::endl;
                }
                /* integer coordinate of lower left corner of post */
                lb[k] = static_cast<abs_type>((left-lower_left[k]+eta[k]/2.0)/eta[k]);
                /* integer coordinate of upper right corner of post */
                ub[k] = static_cast<abs_type>((right-lower_left[k]+eta[k]/2.0)/eta[k]);
                /* number of grid points in the post in each dimension */
                no[k]=(ub[k]-lb[k]+1);
                /* total number of post */
                npost*=no[k];
                cc[k]=0;
            }

            /* compute indices of post */
            for(abs_type k=0; k<npost; k++) {
                abs_type q=0;
                for(int l=0; l<dim; l++)  {
                    q+=(lb[l]+cc[l])*ss.get_nn()[l];
                }
                cc[0]++;
                for(int l=0; l<dim-1; l++) {
                    if(cc[l]==no[l]) {
                        cc[l]=0;
                        cc[l+1]++;
                    }
                }
                if(inside_of_area[q]==0){
                    LtoG.push_back(q);
                    state_type x;
                    ss.itox(q,x);
                    GtoL[q]=index;
                    inside_of_area[q]=1;
                    index++;
                }
            }
        }

        std::cout<<"Full size: "<< ss.size()<<" Reduced size: "<<index<<std::endl;
    }








    /*This function is doing local abstraction*/
    template<class F1, class F2>
    void compute_gb_local(TransitionFunction& transition_function,
                          F1& system_post,
                          F2& radius_post,
                          bool* inside_of_area,
                          std::vector<abs_type>& LtoG,
                          unsigned int* GtoL
                          ){ //__________________________1
        /* number of cells */
        abs_type N=LtoG.size()+1; //__________________________2
        /* number of inputs */
        abs_type M=m_input_alphabet.size();
        /* number of transitions (to be computed) */
        abs_ptr_type T=0;
        /* state space dimension */
        int dim=m_state_alphabet.get_dim();
        /* for display purpose */
        abs_type counter=0;
        /* some grid information */
        std::vector<abs_type> NN=m_state_alphabet.get_nn();

        /* radius of hyper interval containing the attainable set */
        state_type eta;

        /* for out of bounds check */
        state_type lower_left;
        state_type upper_right;
        /* copy data from m_state_alphabet */
        for(int i=0; i<dim; i++) {
            eta[i]=m_state_alphabet.get_eta()[i];
            lower_left[i]=m_state_alphabet.get_lower_left()[i];
            upper_right[i]=m_state_alphabet.get_upper_right()[i];
        }
        /* init in transition_function the members no_pre, no_post, pre_ptr */
        transition_function.init_infrastructure(N,M);
        /* lower-left & upper-right corners of hyper rectangle of cells that cover attainable set */
        std::unique_ptr<abs_type[]> corner_IDs(new abs_type[N*M*2]());
        /* is post of (i,j) out of domain ? */
        bool* out_of_domain=new bool[N*M];
        //std::fill_n(out_of_domain, N*M, 1);

        /* *** Parallel block begins *** */
        /* the variable counter is shared between the threads */
#pragma omp parallel shared(counter)
        {
            /* the following variables are initialized as private copies to each worker */
            /* variables for managing the post */
            std::vector<abs_type> lb(dim);  /* lower-left corner */
            std::vector<abs_type> ub(dim);  /* upper-right corner */
            std::vector<abs_type> no(dim);  /* number of cells per dim */
            std::vector<abs_type> cc(dim);  /* coordinate of current cell in the post */
            /* state and input variables */
            state_type x;
            input_type u;
            /* radius of the hyper-intervals */
            state_type r;

            /*
         * first loop: compute corner_IDs:
         * corner_IDs[i*M+j][0] = lower-left cell index of over-approximation of attainable set
         * corner_IDs[i*M+j][1] = upper-right cell index of over-approximation of attainable set
         */
            /* loop over all cells */
#pragma omp for schedule(dynamic, CHUNK_SIZE)
            for(abs_type i=0; i<N; i++) {

                /* is i an element of the avoid symbols ? */
                /* if(avoid(LtoG[i])) {
            for(abs_type j=0; j<M; j++) {
              out_of_domain[i*M+j]=true;
            }
            continue;
          }*/
                if(i==(N-1)){
                    for(abs_type j=0; j<M; j++) {
                        out_of_domain[i*M+j]=true;
                    }
                    continue;
                }

                /* loop over all inputs */
                for(abs_type j=0; j<M; j++) {
                    out_of_domain[i*M+j]=false;
                    /* get center x of cell */
                    m_state_alphabet.itox(LtoG[i],x);//__________________________3
                    /* cell radius (including measurement errors) */
                    for(int k=0; k<dim; k++)
                        r[k]=eta[k]/2.0+m_z[k];
                    /* current input */
                    m_input_alphabet.itox(j,u);
                    /* integrate system and radius growth bound */
                    /* the result is stored in x and r */
                    radius_post(r,x,u);
                    system_post(x,u);
                    /* determine the cells which intersect with the attainable set:
             * discrete hyper interval of cell indices
             * [lb[0]; ub[0]] x .... x [lb[dim-1];left ub[dim-1]]
             * covers attainable set
             */
                    abs_type npost=1;

                    state_type Left;
                    state_type Right;
                    bool out=false;


                    for(int k=0; k<dim; k++) {
                        double left = x[k]-r[k]-m_z[k];
                        double right = x[k]+r[k]+m_z[k];
                        if(left <= lower_left[k]-eta[k]/2.0  || right >= upper_right[k]+eta[k]/2.0)  {
                            out_of_domain[i*M+j]=true;
                            break;
                        }
                        Left[k]=left;
                        Right[k]=right;

                    }
                    if((inside_of_area[m_state_alphabet.xtoi(Left)]==0) || (inside_of_area[m_state_alphabet.xtoi(Right)]==0)){
                        out_of_domain[i*M+j]=true;
                    }

                    if(out_of_domain[i*M+j])
                        continue;

                    for(int k=0; k<dim; k++) {
                        /* check for out of bounds */


                        /* integer coordinate of lower left corner of post */
                        lb[k] = static_cast<abs_type>((Left[k]-lower_left[k]+eta[k]/2.0)/eta[k]);
                        /* integer coordinate of upper right corner of post */
                        ub[k] = static_cast<abs_type>((Right[k]-lower_left[k]+eta[k]/2.0)/eta[k]);
                        /* number of grid points in the post in each dimension */
                        no[k]=(ub[k]-lb[k]+1);
                        /* total number of post */
                        npost*=no[k];
                        cc[k]=0;
                    }

                    corner_IDs[i*(2*M)+2*j]=0;
                    corner_IDs[i*(2*M)+2*j+1]=0;


                    /* compute indices of post */
                    for(abs_type k=0; k<npost; k++) {  //_________NO CHANGE
                        abs_type q=0;
                        for(int l=0; l<dim; l++)
                            q+=(lb[l]+cc[l])*NN[l];
                        cc[0]++;
                        for(int l=0; l<dim-1; l++) {
                            if(cc[l]==no[l]) {
                                cc[l]=0;
                                cc[l+1]++;
                            }
                        }
                        /* (i,j,q) is a transition */
                        /* increment number of pres for (q,j) */
                        if(inside_of_area[q]==0 ){
                            out_of_domain[i*M+j]=true;
                            for(int b=0;b<dim;b++)
                                cc[b]=0;
                            for(abs_type z=0; z<k-1; z++) {  //_________NO CHANGE
                                abs_type q=0;
                                for(int l=0; l<dim; l++)
                                    q+=(lb[l]+cc[l])*NN[l];
                                cc[0]++;
                                for(int l=0; l<dim-1; l++) {
                                    if(cc[l]==no[l]) {
                                        cc[l]=0;
                                        cc[l+1]++;
                                    }
                                }
                                transition_function.m_no_pre[GtoL[q]*M+j]--;
                            }

                            break;
                        }

#pragma omp atomic
                        transition_function.m_no_pre[GtoL[q]*M+j]++;
                        /* store id's of lower-left and upper-right cell */

                    }
                    if(out_of_domain[i*M+j])
                        continue;
                    corner_IDs[i*(2*M)+2*j]=m_state_alphabet.xtoi(Left );
                    corner_IDs[i*(2*M)+2*j+1]=m_state_alphabet.xtoi(Right );


                    /* increment number of transitions by number of post */
#pragma omp atomic
                    T+=npost;
                    transition_function.m_no_post[i*M+j]=npost;
                }
                /* print progress */
                if(m_verbose) {
                    if(counter==0)
                        std::cout << "1st loop: ";
                }

                progress(i,N,counter);
            } /* end of for loop over the states */
        } /* end of parallel block */
        /* compute pre_ptr */
        abs_ptr_type sum=0;
        for(abs_type i=0; i<N; i++) {
            for(abs_type j=0; j<M; j++) {
                sum+=transition_function.m_no_pre[i*M+j];
                transition_function.m_pre_ptr[i*M+j]=sum;
            }
        }
        /* allocate memory for pre list */
        transition_function.init_transitions(T);
        std::cout<<std::endl<<"T number: "<<T<<std::endl;
        /* second loop: fill pre array */
        counter=0;
        /* *** Parallel block begins *** */
        /* the variable counter is shared between the threads */
#pragma omp parallel shared(counter)
        {
            /* the following variables are initialized as private copies to each worker */
            /* variables for managing the post */
            std::vector<abs_type> lb(dim);  /* lower-left corner */
            std::vector<abs_type> ub(dim);  /* upper-right corner */
            std::vector<abs_type> no(dim);  /* number of cells per dim */
            std::vector<abs_type> cc(dim);  /* coordinate of current cell in the post */
#pragma omp for schedule(dynamic, CHUNK_SIZE)
            for(abs_type i=0; i<N; i++) {
                /* loop over all inputs */
                for(abs_type j=0; j<M; j++) {

                    /* is x an element of the overflow symbols ? */
                    if(out_of_domain[i*M+j])
                        continue;
                    /* extract lower-left and upper-bound points */
                    abs_type k_lb=corner_IDs[i*2*M+2*j];
                    abs_type k_ub=corner_IDs[i*2*M+2*j+1];
                    abs_type npost=1;

                    /* cell idx to coordinates */
                    for(int k=dim-1; k>=0; k--) {
                        /* integer coordinate of lower left corner */
                        lb[k]=k_lb/NN[k];
                        k_lb=k_lb-lb[k]*NN[k];
                        /* integer coordinate of upper right corner */
                        ub[k]=k_ub/NN[k];
                        k_ub=k_ub-ub[k]*NN[k];
                        /* number of grid points in each dimension in the post */
                        no[k]=(ub[k]-lb[k]+1);
                        /* total no of post of (i,j) */
                        npost*=no[k];
                        cc[k]=0;

                    }



                    for(abs_type k=0; k<npost; k++) {
                        abs_type q=0;
                        for(int l=0; l<dim; l++)
                            q+=(lb[l]+cc[l])*NN[l];
                        cc[0]++;
                        for(int l=0; l<dim-1; l++) {
                            if(cc[l]==no[l]) {
                                cc[l]=0;
                                cc[l+1]++;
                            }
                        }

                        //              if(q>m_state_alphabet.size()){
                        //                continue;
                        //              }
                        /* (i,j,q) is a transition */
                        //              if(inside_of_area[q]==0 ){
                        //              //    abs_type TempXtoi=(N-1)*M+j;
                        //              //    if(transition_function.m_pre_ptr[TempXtoi]>0)
                        //              //      transition_function.m_pre[--transition_function.m_pre_ptr[TempXtoi]]=i;
                        //                  continue;
                        //              }


                        abs_type TempXtoi=GtoL[q]*M+j;
                        //if(transition_function.m_pre_ptr[TempXtoi]>0)
                        transition_function.m_pre[--transition_function.m_pre_ptr[TempXtoi]]=i;
                    }
                }
                /* print progress */
                if(m_verbose) {
                    if(counter==0)
                        std::cout << "2nd loop: ";
                }
                progress(i,N,counter);
            } /* end of for over states */
        } /* end of parallel block */
    }












    /*This function is doing Global abstraction*/
    template<class F1, class F2, class F3=decltype(params::avoid_abs)>
    void compute_gb_global(TransitionFunction& transition_function,
                    F1& system_post,
                    F2& radius_post,
                    F3& avoid
                    ){
        //std::unordered_set<abs_type>  area) {
        /* number of cells */
        abs_type N=m_state_alphabet.size();
        /* number of inputs */
        abs_type M=m_input_alphabet.size();
        /* number of transitions (to be computed) */
        abs_ptr_type T=0;
        /* state space dimension */
        int dim=m_state_alphabet.get_dim();
        /* for display purpose */
        abs_type counter=0;
        /* some grid information */
        std::vector<abs_type> NN=m_state_alphabet.get_nn();
        /* variables for managing the post */

        /* radius of hyper interval containing the attainable set */
        state_type eta;

        /* for out of bounds check */
        state_type lower_left;
        state_type upper_right;
        /* copy data from m_state_alphabet */
        for(int i=0; i<dim; i++) {
            eta[i]=m_state_alphabet.get_eta()[i];
            lower_left[i]=m_state_alphabet.get_lower_left()[i];
            upper_right[i]=m_state_alphabet.get_upper_right()[i];
        }
        /* init in transition_function the members no_pre, no_post, pre_ptr */
        transition_function.init_infrastructure(N,M);
        /* lower-left & upper-right corners of hyper rectangle of cells that cover attainable set */
        std::unique_ptr<abs_type[]> corner_IDs(new abs_type[N*M*2]());
        /* is post of (i,j) out of domain ? */
        bool* out_of_domain=new bool[N*M];
        //std::fill_n(out_of_domain, N*M, 1);
#pragma omp parallel shared(counter)
        {
            std::vector<abs_type> lb(dim);  /* lower-left corner */
            std::vector<abs_type> ub(dim);  /* upper-right corner */
            std::vector<abs_type> no(dim);  /* number of cells per dim */
            std::vector<abs_type> cc(dim);  /* coordinate of current cell in the post */

            state_type r;
            /* state and input variables */
            state_type x;
            input_type u;
            /*
   * first loop: compute corner_IDs:
   * corner_IDs[i*M+j][0] = lower-left cell index of over-approximation of attainable set
   * corner_IDs[i*M+j][1] = upper-right cell index of over-approximation of attainable set
   */
            /* loop over all cells */
#pragma omp for schedule(dynamic, CHUNK_SIZE)
            for(abs_type i=0; i<N; i++) {
                //for (const auto i:area) {

                /* is i an element of the avoid symbols ? */
                if(avoid(i)) {
                    for(abs_type j=0; j<M; j++) {
                        out_of_domain[i*M+j]=true;
                    }
                    continue;
                }
                /*    if(inside_of_area[i]==0){
        for(abs_type j=0; j<M; j++) {
          out_of_domain[i*M+j]=true;
        }
        continue;
    }*/
                /* loop over all inputs */
                for(abs_type j=0; j<M; j++) {
                    out_of_domain[i*M+j]=false;
                    /* get center x of cell */
                    m_state_alphabet.itox(i,x);
                    /* cell radius (including measurement errors) */
                    for(int k=0; k<dim; k++)
                        r[k]=eta[k]/2.0+m_z[k];
                    /* current input */
                    m_input_alphabet.itox(j,u);
                    /* integrate system and radius growth bound */
                    /* the result is stored in x and r */
                    radius_post(r,x,u);
                    system_post(x,u);
                    /* determine the cells which intersect with the attainable set:
       * discrete hyper interval of cell indices
       * [lb[0]; ub[0]] x .... x [lb[dim-1]; ub[dim-1]]
       * covers attainable set
       */
                    abs_type npost=1;
                    for(int k=0; k<dim; k++) {
                        /* check for out of bounds */
                        double left = x[k]-r[k]-m_z[k];
                        double right = x[k]+r[k]+m_z[k];
                        if(left <= lower_left[k]-eta[k]/2.0  || right >= upper_right[k]+eta[k]/2.0)  {
                            out_of_domain[i*M+j]=true;
                            break;
                        }

                        /* integer coordinate of lower left corner of post */
                        lb[k] = static_cast<abs_type>((left-lower_left[k]+eta[k]/2.0)/eta[k]);
                        /* integer coordinate of upper right corner of post */
                        ub[k] = static_cast<abs_type>((right-lower_left[k]+eta[k]/2.0)/eta[k]);
                        /* number of grid points in the post in each dimension */
                        no[k]=(ub[k]-lb[k]+1);
                        /* total number of post */
                        npost*=no[k];
                        cc[k]=0;
                    }
                    corner_IDs[i*(2*M)+2*j]=0;
                    corner_IDs[i*(2*M)+2*j+1]=0;
                    if(out_of_domain[i*M+j])
                        continue;

                    /* compute indices of post */
                    for(abs_type k=0; k<npost; k++) {
                        abs_type q=0;
                        for(int l=0; l<dim; l++)
                            q+=(lb[l]+cc[l])*NN[l];
                        cc[0]++;
                        for(int l=0; l<dim-1; l++) {
                            if(cc[l]==no[l]) {
                                cc[l]=0;
                                cc[l+1]++;
                            }
                        }
                        /* (i,j,q) is a transition */
                        /* increment number of pres for (q,j) */
#pragma omp atomic
                        transition_function.m_no_pre[q*M+j]++;
                        /* store id's of lower-left and upper-right cell */
                        if(k==0)
                            corner_IDs[i*(2*M)+2*j]=q;
                        if(k==npost-1)
                            corner_IDs[i*(2*M)+2*j+1]=q;
                    }
                    /* increment number of transitions by number of post */
#pragma omp atomic
                    T+=npost;
                    transition_function.m_no_post[i*M+j]=npost;
                }
                /* print progress */
                if(m_verbose) {
                    if(counter==0)
                        std::cout << "1st loop: ";
                }
                progress(i,N,counter);
            }
        }
        /* compute pre_ptr */
        abs_ptr_type sum=0;
        for(abs_type i=0; i<N; i++) {
            //for (const auto i:area) {
            for(abs_type j=0; j<M; j++) {
                sum+=transition_function.m_no_pre[i*M+j];
                transition_function.m_pre_ptr[i*M+j]=sum;
            }
        }
        /* allocate memory for pre list */
        transition_function.init_transitions(T);

        /* second loop: fill pre array */
        counter=0;
#pragma omp parallel shared(counter)
        {

            std::vector<abs_type> lb(dim);  /* lower-left corner */
            std::vector<abs_type> ub(dim);  /* upper-right corner */
            std::vector<abs_type> no(dim);  /* number of cells per dim */
            std::vector<abs_type> cc(dim);  /* coordinate of current cell in the post */
#pragma omp for schedule(dynamic, CHUNK_SIZE)
            for(abs_type i=0; i<N; i++) {
                //for (const auto i:area) {
                /* loop over all inputs */
                for(abs_type j=0; j<M; j++) {
                    /* is x an element of the overflow symbols ? */
                    if(out_of_domain[i*M+j])
                        continue;
                    /* extract lower-left and upper-bound points */
                    abs_type k_lb=corner_IDs[i*2*M+2*j];
                    abs_type k_ub=corner_IDs[i*2*M+2*j+1];
                    abs_type npost=1;

                    /* cell idx to coordinates */
                    for(int k=dim-1; k>=0; k--) {
                        /* integer coordinate of lower left corner */
                        lb[k]=k_lb/NN[k];
                        k_lb=k_lb-lb[k]*NN[k];
                        /* integer coordinate of upper right corner */
                        ub[k]=k_ub/NN[k];
                        k_ub=k_ub-ub[k]*NN[k];
                        /* number of grid points in each dimension in the post */
                        no[k]=(ub[k]-lb[k]+1);
                        /* total no of post of (i,j) */
                        npost*=no[k];
                        cc[k]=0;

                    }

                    for(abs_type k=0; k<npost; k++) {
                        abs_type q=0;
                        for(int l=0; l<dim; l++)
                            q+=(lb[l]+cc[l])*NN[l];
                        cc[0]++;
                        for(int l=0; l<dim-1; l++) {
                            if(cc[l]==no[l]) {
                                cc[l]=0;
                                cc[l+1]++;
                            }
                        }
                        /* (i,j,q) is a transition */
                        transition_function.m_pre[--transition_function.m_pre_ptr[q*M+j]]=i;
                    }
                }
                /* print progress */
                if(m_verbose) {
                    if(counter==0)
                        std::cout << "2nd loop: ";
                }
                progress(i,N,counter);
            }
        }
    }








    /** @brief get the center of cells that are used to over-approximated the
   *  attainable set associated with cell (with center x) and input u
   *
   *  @returns std::vector<abs_type>
   *  @param system_post - lambda expression as in compute_gb
   *  @param radius_post - lambda expression as in compute_gb
   *  @param x - center of cell
   *  @param u - input
   **/
    template<class F1, class F2>
    std::vector<state_type> get_post(F1& system_post,
                                     F2& radius_post,
                                     const state_type& x,
                                     const input_type& u) const {
        /* initialize return value */
        std::vector<state_type> post {};
        /* state space dimension */
        int dim = m_state_alphabet.get_dim();
        /* variables for managing the post */
        std::vector<abs_type> lb(dim);  /* lower-left corner */
        std::vector<abs_type> ub(dim);  /* upper-right corner */
        std::vector<abs_type> no(dim);  /* number of cells per dim */
        std::vector<abs_type> cc(dim);  /* coordinate of current cell in the post */
        /* get radius */
        state_type r;
        state_type eta;
        /* for out of bounds check */
        state_type lower_left;
        state_type upper_right;
        /* fill in data */
        for(int i=0; i<dim; i++) {
            eta[i]=m_state_alphabet.get_eta()[i];
            r[i]=m_state_alphabet.get_eta()[i]/2.0+m_z[i];
            lower_left[i]=m_state_alphabet.get_lower_left()[i];
            upper_right[i]=m_state_alphabet.get_upper_right()[i];
        }
        state_type xx=x;
        /* compute growth bound and numerical solution of ODE */
        radius_post(r,x,u);
        system_post(xx,u);

        /* determine post */
        abs_type npost=1;
        for(int k=0; k<dim; k++) {
            /* check for out of bounds */
            double left = xx[k]-r[k]-m_z[k];
            double right = xx[k]+r[k]+m_z[k];
            if(left <= lower_left[k]-eta[k]/2.0  || right >= upper_right[k]+eta[k]/2.0) {
                return post;
            }
            /* integer coordinate of lower left corner of post */
            lb[k] = static_cast<abs_type>((left-lower_left[k]+eta[k]/2.0)/eta[k]);
            /* integer coordinate of upper right corner of post */
            ub[k] = static_cast<abs_type>((right-lower_left[k]+eta[k]/2.0)/eta[k]);
            /* number of grid points in the post in each dimension */
            no[k]=(ub[k]-lb[k]+1);
            /* total number of post */
            npost*=no[k];
            cc[k]=0;
        }
        /* compute indices of post */
        for(abs_type k=0; k<npost; k++) {
            abs_type q=0;
            for(int l=0; l<dim; l++)  {
                q+=(lb[l]+cc[l])*m_state_alphabet.get_nn()[l];
            }
            cc[0]++;
            for(int l=0; l<dim-1; l++) {
                if(cc[l]==no[l]) {
                    cc[l]=0;
                    cc[l+1]++;
                }
            }
            /* (i,j,q) is a transition */
            m_state_alphabet.itox(q,xx);
            post.push_back(xx);
        }
        return post;
    }


    /** @brief print the center of cells that are stored in the transition
   *  function as post of the cell with center x and input u
   *
   *  @param transition_function - the transition function of the abstraction
   *  @param x - center of cell
   *  @param u - input
   **/
    void print_post(const TransitionFunction& transition_function,
                    const state_type& x,
                    const input_type& u) const {
        std::vector<state_type> post = get_post(transition_function, x, u);
        if(!post.size())
            std::cout << "\nPost is out of domain\n";
        std::cout << "\nPost states: \n";
        for(abs_type v=0; v<post.size(); v++) {
            for(int i=0; i<m_state_alphabet.get_dim(); i++) {
                std::cout << post[v][i] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    /** @brief print the center of cells that are used to over-approximated the
   *  attainable set (computed according to the growth bound)
   *  associated with the cell with center x and input u
   *
   *  @param system_post - lambda expression as in compute_gb
   *  @param radius_post - lambda expression as in compute_gb
   *  @param x - center of cell
   *  @param u - input
   **/
    template<class F1, class F2>
    void print_post_gb(F1& system_post,
                       F2& radius_post,
                       const state_type& x,
                       const input_type& u) const {
        std::vector<state_type> post = get_post(system_post,radius_post,x,u);
        if(!post.size()) {
            std::cout << "\nPost is out of domain\n";
            return;
        }

        std::cout << "\nPost states: \n";
        for(abs_type v=0; v<post.size(); v++) {
            for(int i=0; i<m_state_alphabet.get_dim(); i++) {
                std::cout << post[v][i] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    /** @brief set the measurement error bound **/
    void set_measurement_error_bound(const state_type& error_bound) {
        for(int i=0; i<m_state_alphabet.get_dim(); i++) {
            m_z[i]=error_bound[i];
        }
    }
    /** @brief get measurement error bound **/
    std::vector<double> get_measruement_error_bound() {
        std::vector<double> z;
        for(int i=0; i<m_state_alphabet.get_dim(); i++) {
            z.push_back(m_z[i]);
        }
        return z;
    }
    /** @brief activate console output **/
    void verbose_on() {
        m_verbose=true;
    }
    /** @brief deactivate console output **/
    void verbose_off() {
        m_verbose=false;
    }
};

} /* close namespace */
#endif /* ABSTRACTION_HH_ */
