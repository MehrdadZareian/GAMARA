
import Pkg; Pkg.activate(@__DIR__); Pkg.instantiate();
#Write Altro folder instead of @__DIR__
using TrajectoryOptimization
using Plots, LinearAlgebra


#number individual systems
agent_number=10

#defining product system dynamics
function dynamics!(ẋ::AbstractVector{T},x::AbstractVector{T},u::AbstractVector{T}) where T

	for k=1:10
		ẋ[3*k-2] = u[2*k-1]*cos(x[3*k]);
		ẋ[3*k-1] = u[2*k-1]*sin(x[3*k]);
	    ẋ[3*k] = u[2*k];
	end
end


#Total number of states
n=3*agent_number
#Total number of control inputs
m=2*agent_number


#defining descrete model
model = Model(dynamics!,n,m)
model_d = rk4(model)
n = model.n;
m = model.m;


T = Float64;


#inital point for product system
x0 = [0.0;2.0;0.0;
0.0;4.0;0.0;
0.0;6.0;0.0;
0.0;8.0;0.0;
2.0;0.0;pi/2;
4.0;0.0;pi/2;
6.0;0.0;pi/2;
8.0;0.0;pi/2;
0.0;0.0;pi/4;
0.0;10.0;-pi/4]

#goal point for product system
xf = [10.0;2.0;0.0;
10.0;4.0;0.0;
10.0;6.0;0.0;
10.0;8.0;0.0;
2.0;10.0;pi/2;
4.0;10.0;pi/2;
6.0;10.0;pi/2;
8.0;10.0;pi/2;
10.0;10.0;pi/4;
10.0;0.0;-pi/4]



#number of points in trajectory(number of time samples)
N = 104
#sampling time
dt = 0.1
#total time
tf = (N-1)*dt

#LQR Cost Matrices (same as default)
Q = 0.01*Diagonal(I,n)
Qf = 1000.0*Diagonal(I,n)
R = 0.01*Diagonal(I,m)
obj = LQRObjective(Q,R,Qf,xf,N)


#creating upperbound and lower bound for control input with bound constraint
constraints = Constraints(N)
u_min = Array{Float64}(undef,2*agent_number)
u_max = Array{Float64}(undef,2*agent_number)
u_c=2;
for k=1:2*agent_number
	u_min[k]=-u_c;
	u_max[k]=u_c;
end
bnd = BoundConstraint(n,m,u_min=u_min,u_max=u_max)
for k = 1:N-1
     constraints[k] += bnd
 end


 #constraint for obstacle
 #this function is defining penalty for collision constraint
function My_rectangle(x)
	M=10000
	ϵ=1.46
		for i=1:10
        	temp=max((x[3*i-2]-5)^2,(x[3*i-1]-5)^2)
			M=min(temp,M)
    	end
    return -(M-ϵ^2);
end

#constraint for obstacle
#this function is defining penalty for collision constraint
function My_constraint(x)
	ϵ=0.64
	M=1000;
	for i=1:9 #agent number -1
		for j=i+1:10  #agent number
		temp=max((x[3*i-2]-x[3*j-2])^2 , (x[3*i-1]-x[3*j-1])^2);
		M=min(temp,M);
		end
	end
	return -(M - ϵ^2)
end

function My_obs(c,x,u)
    c[1]=My_constraint(x);
    c[2]=My_rectangle(x);
	return nothing
end

obs= Constraint{Inequality}(My_obs,n,m,2,:obs)
for k = 1:N-1
     constraints[k] += obs
 end

#goal constraint
goal = goal_constraint(xf);
constraints[N] += goal

prob = Problem(model_d,obj,constraints=constraints,x0=x0,N=N,dt=dt,xf=xf)

# options for solver
max_con_viol = 1.0e-8
verbose=false

opts_ilqr = iLQRSolverOptions{T}(verbose=verbose,
    iterations=300)

opts_al = AugmentedLagrangianSolverOptions{T}(verbose=verbose,
    opts_uncon=opts_ilqr,
    iterations=40,
    cost_tolerance=1.0e-5,
    cost_tolerance_intermediate=1.0e-4,
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

	solver = AugmentedLagrangianSolver(prob)

	#here we call the solver to solve
    @time begin
    a=solve!(prob, opts_altro); # solve with ALTRO
    end


#parsing the solution to differenet arrays

	x=Array{Array{Float64,1}}(undef,agent_number)
	z=Array{Array{Float64,1}}(undef,agent_number)
	for i=1:agent_number
    	x[i] = [prob.X[k][3*i-2] for k = 1:N]
    	z[i] = [prob.X[k][3*i-1] for k = 1:N]
	end
#plotting
	a=plot()

	rectangle(w, h, x, y) = Shape(x .+ [0,w,w,0], y .+ [0,0,h,h])
	plot!(rectangle(2,2,4,4), opacity=.5)
	plot!(rectangle(2.8,2.8,3.6,3.6), opacity=.3)

	plot!(x[1],z[1],xlabel="x",ylabel="y",label="1",legend=:topleft,color=:blue,width=2,ratio=:equal,title="Multi-Agent Example Trajectories")
	plot!(x[2],z[2],xlabel="x",ylabel="y",label="2",legend=:topleft,width=2,ratio=:equal)
	plot!(x[3],z[3],xlabel="x",ylabel="y",label="3",legend=:topleft,width=2,ratio=:equal)
	plot!(x[4],z[4],xlabel="x",ylabel="y",label="4",legend=:topleft,width=2,ratio=:equal)
	plot!(x[5],z[5],xlabel="x",ylabel="y",label="5",legend=:topleft,width=2,ratio=:equal)
	plot!(x[6],z[6],xlabel="x",ylabel="y",label="6",legend=:topleft,width=2,ratio=:equal)
	plot!(x[7],z[7],xlabel="x",ylabel="y",label="7",legend=:topleft,width=2,ratio=:equal)
	plot!(x[8],z[8],xlabel="x",ylabel="y",label="8",legend=:topleft,width=2,ratio=:equal)
	plot!(x[9],z[9],xlabel="x",ylabel="y",label="9",legend=:topleft,width=2,ratio=:equal)
	plot!(x[10],z[10],xlabel="x",ylabel="y",label="10",legend=:topleft,width=2,ratio=:equal)


	plot!((x[1][1],z[1][1]),marker=:circle,color=:red,label="")
    plot!((x[1][end],z[1][end]),marker=:utriangle,color=:green,label="")

	plot!((x[2][1],z[2][1]),marker=:circle,color=:red,label="")
    plot!((x[2][end],z[2][end]),marker=:utriangle,color=:green,label="")

	plot!((x[3][1],z[3][1]),marker=:circle,color=:red,label="")
    plot!((x[3][end],z[3][end]),marker=:utriangle,color=:green,label="")

	plot!((x[4][1],z[4][1]),marker=:circle,color=:red,label="")
    plot!((x[4][end],z[4][end]),marker=:utriangle,color=:green,label="")

	plot!((x[5][1],z[5][1]),marker=:circle,color=:red,label="")
    plot!((x[5][end],z[5][end]),marker=:utriangle,color=:green,label="")

	plot!((x[6][1],z[6][1]),marker=:circle,color=:red,label="")
    plot!((x[6][end],z[6][end]),marker=:utriangle,color=:green,label="")

	plot!((x[7][1],z[7][1]),marker=:circle,color=:red,label="")
    plot!((x[7][end],z[7][end]),marker=:utriangle,color=:green,label="")

	plot!((x[8][1],z[8][1]),marker=:circle,color=:red,label="")
    plot!((x[8][end],z[8][end]),marker=:utriangle,color=:green,label="")

	plot!((x[9][1],z[9][1]),marker=:circle,color=:red,label="")
    plot!((x[9][end],z[9][end]),marker=:utriangle,color=:green,label="")

	plot!((x[10][1],z[10][1]),marker=:circle,color=:red,label="")
    plot!((x[10][end],z[10][end]),marker=:utriangle,color=:green,label="")

	io = open("nom_tr.txt", "w")
	for i in prob.U
		for k=1:m-1
    		print(io,i[k])
    		print(io," ")
		end
    	println(io,i[m])
	end
	close(io)
