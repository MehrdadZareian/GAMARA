using TrajectoryOptimization
using Plots, LinearAlgebra




#number individual systems
function cart_pole!(ẋ::AbstractVector{T},x::AbstractVector{T},u::AbstractVector{T}) where T
    g = 9.8;
    M_c=1.0;
    M_p=0.1;
    M_t=M_p+M_c;
    l=0.5;
    c1 = u[1]/M_t;
    c2 = M_p*l/M_t;

    c3=l*4/3;
    c4=l*M_p/M_t;

    F=(g*sin(x[3])-cos(x[3])*(c1+c2*x[4]*x[4]*sin(x[3])))/(c3-c4*cos(x[3])*cos(x[3]));
    G= (c1+c2*x[4]*x[4]*sin(x[3])) - c4* cos(x[3])*F;

	ẋ[1] = x[2];
	ẋ[2] = G;
	ẋ[3]= x[4];
    ẋ[4] = F;

	ẋ[5]=x[6];
	ẋ[6]=u[2];
end
#total number of states
n=6
#total number of control inputs
m=2

#defining descrete model
model = Model(cart_pole!,n,m)
model_d = rk4(model)
n = model.n;
m = model.m;

T = Float64;

#inital point for product system
x0 = [0.0;0.0;pi;0.0;8.0;0.0]
#Total number of control inputs
xf = [5.0;0.0;pi;0.0;4.0;0.0]

N = 70
dt = 0.1
tf = (N-1)*dt


Q = 0.01*Diagonal(I,n)
Qf = 1000.0*Diagonal(I,n)
R = 0.01*Diagonal(I,m)
obj = LQRObjective(Q,R,Qf,xf,N)
constraints = Constraints(N)


u_min=[-6.0,-5.0]
u_max=[6.0,5.0]

bnd = BoundConstraint(n,m,u_min=u_min,u_max=u_max)

for k = 1:N-1
     constraints[k] += bnd
 end

#Collision constraint
#this function defines constraint penalty
function My_constraint(x)
	ϵ=0.35;
	l=0.5;
	X1=x[1]+l*sin(x[3]);
	Y1=0.6+l*cos(x[3]);
	X2=x[5]
	Y2=0.2;
	dist=max((X1-X2)^2 , (Y1-Y2)^2)
	return -(dist - ϵ^2)
end
function My_obs(c,x,u)
    c[1]=My_constraint(x);
    c[2]=-1;
	return nothing
end

obs= Constraint{Inequality}(My_obs,n,m,2,:obs)
for k = 1:N-1
     constraints[k] += obs
 end

# goal constraint
goal = goal_constraint(xf);
constraints[N] += goal

prob = Problem(model_d,obj,constraints=constraints,x0=x0,N=N,dt=dt,xf=xf)

# options
max_con_viol = 1.0e-9
verbose=false

opts_ilqr = iLQRSolverOptions{T}(verbose=verbose,
    iterations=300)

opts_al = AugmentedLagrangianSolverOptions{T}(verbose=verbose,
    opts_uncon=opts_ilqr,
    iterations=40,
    cost_tolerance=1.0e-6,
    cost_tolerance_intermediate=1.0e-6,
    constraint_tolerance=max_con_viol,
    penalty_scaling=10.,
    penalty_initial=1.)

opts_pn = ProjectedNewtonSolverOptions{T}(verbose=verbose,
    feasibility_tolerance=max_con_viol,
    solve_type=:feasible)

opts_altro = ALTROSolverOptions{T}(verbose=verbose,
    opts_al=opts_al,
    R_inf=1.0e-8,
    resolve_feasible_problem=false,
    opts_pn=opts_pn,
    projected_newton=true,
    projected_newton_tolerance=1.0e-4);

    @time begin
    solve!(prob, opts_altro); # solve with ALTRO
    end


	io = open("/home/mehrdad/Scots+Altro2/examples/cart_pole_MA/nom_tr.txt", "w")
	for i in prob.U
		print(io,i[1])
		print(io," ")
		println(io,i[2])
	end
	close(io)


#=	io = open("/home/mehrdad/Scots+Altro/examples/unicycle1D/uni_trajectory5.txt", "w")
    for i in prob.U
        println(io,i[2])
    end
    close(io)
=#



    x1 = [prob.X[k][1] for k = 1:N]
	x2 = [prob.X[k][2] for k = 1:N]
	x3 = [prob.X[k][3] for k = 1:N]
	x4 = [prob.X[k][4] for k = 1:N]
	x5 = [prob.X[k][5] for k = 1:N]
	x6 = [prob.X[k][6] for k = 1:N]



	X1=Array{Float64,1}(undef,N)
	X2=x5
	Y1=Array{Float64,1}(undef,N)
	Y2=Array{Float64,1}(undef,N)
	Y3=Array{Float64,1}(undef,N)
	dist=Array{Float64,1}(undef,N);
	l=0.5
	for i = 1:N
		X1[i]=x1[i]+l*sin(x3[i]);
		Y1[i]=0.6+l*cos(x3[i]);

		Y2[i]=0.2;
		Y3[i]=0.6;


		if(max((X1[i]-X2[i])^2 , (Y1[i]-Y2[i])^2) < 0.09)
			println("error")
		end
	end

	c=plot(dist)


rectangle(w, h, x, y) = Shape(x .+ [0,w,w,0], y .+ [0,0,h,h])
