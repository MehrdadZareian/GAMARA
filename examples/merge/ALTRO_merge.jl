
import Pkg; Pkg.activate(@__DIR__); Pkg.instantiate();
#Write Altro folder instead of @__DIR__

using TrajectoryOptimization
using Plots, LinearAlgebra



#number individual systems
agent_number=6;

#defining product system dynamics
function dynamics!(ẋ::AbstractVector{T},x::AbstractVector{T},u::AbstractVector{T}) where T
	for k=1:agent_number
		ẋ[3*k-2] = u[2*k-1]*cos(x[3*k]);
		ẋ[3*k-1] = u[2*k-1]*sin(x[3*k]);
	    ẋ[3*k] = u[2*k];
	end
end
#Total number of states
n=agent_number*3
#Total number of control inputs
m=agent_number*2

#defining descrete model
model = Model(dynamics!,n,m)
model_d = rk4(model)
n = model.n;
m = model.m;

T = Float64;


#inital point for product system
x0 = [
6.0;1.2;0.0;
3.0;1.2;0.0;
0.0;1.2;0.0;
8;3.9;-0.64;
6;5.4;-0.64;
4.0;6.9;-0.64
]
#goal point for product system
xf = [
17.0;1.2;0.0;
17.0;1.2;0.0;
17.0;1.2;0.0;
17.0;1.2;0.0;
17.0;1.2;0.0;
17.0;1.2;0.0
]

goal = goal_constraint(xf);

#these 2 parameters are very important
#number of points in trajectory(number of time samples)
N = 110
#sampling time
dt = 0.1
#total time
tf = (N-1)*dt

#LQR Cost Matrices (same as default)
Q = 0.01*Diagonal(I,n)
Qf = 1000.0*Diagonal(I,n)
R = 0.01*Diagonal(I,m)
obj = LQRObjective(Q,R,Qf,xf,N)



#defining Constraint for each point in time
constraints = Constraints(N)


#creating upperbound and lower bound for control input with bound constraint
u_min = Array{Float64}(undef,m)
u_max = Array{Float64}(undef,m)
u_c=2;
for k=1:m
	u_min[k]=-u_c;
	u_max[k]=u_c;
end
bnd = BoundConstraint(n,m,u_min=u_min,u_max=u_max)
for k = 1:N-1
    constraints[k] += bnd
 end





#collision constraint

function collision_constraint(x)
	ϵ=1.22;
	M=1000;
	temp=ϵ+0.1
	for i=1:agent_number
		for j=i+1:agent_number
			if((x[3*i-2]<15) &&  (x[3*j-2]<15))
				temp=max((x[3*i-2]-x[3*j-2])^2 , (x[3*i-1]-x[3*j-1])^2);
			else
				continue;
			end
			M=min(temp,M);
		end
	end
	return -(M - ϵ^2)
end


#These 3 constraints are for avoiding the road sides

function constraint1(x)
	M=-10000
	temp=100
	for i=1:agent_number
		if( (3*x[3*i-2]+4*x[3*i-1] <=9.15*4) && (x[3*i-1]>=1.75) )
			temp=min(((3*x[3*i-2]+4*x[3*i-1]-9.15*4)/5)^2,(x[3*i-1]-1.75)^2);
		else
			 temp=-min(((3*x[3*i-2]+4*x[3*i-1]-9.15*4)/5)^2,(x[3*i-1]-1.75)^2) ;
		 end
		 M=max(M,temp);
    end
    return M;
end

function constraint2(x)
	M=-10000
	temp=100
	for i=1:agent_number
		if( (3*x[3*i-2]+4*x[3*i-1] >=10.65*4) && (x[3*i-1]>=1.75))
			temp=min(((3*x[3*i-2]+4*x[3*i-1]-10.65*4)/5)^2 , (x[3*i-1]-1.75)^2)
		else
			temp= -min(((3*x[3*i-2]+4*x[3*i-1]-10.65*4)/5)^2,(x[3*i-1]-1.75)^2);
		 end
		 M=max(M,temp);
    end
    return M
end

function constraint3(x)
	M=-10000
	temp=100
	for i=1:agent_number
		if( (x[3*i-1]<=0.65) )
			temp=(x[3*i-1]-0.65)^2
		else
			 temp= -(x[3*i-1]-0.65)^2;
		 end
		 M=max(M,temp);
    end
    return M;
end

function AllConstraints(c,x,u)
    c[1]=collision_constraint(x);
    c[2]=constraint1(x);
	c[3]=constraint2(x);
	c[4]=constraint3(x);
	return nothing
end
obs= Constraint{Inequality}(AllConstraints,n,m,4,:obs)

for k = 1:N-1
     constraints[k] += obs
end



prob = Problem(model_d,obj,constraints=constraints,x0=x0,N=N,dt=dt,xf=xf)

# options for ALTRO Solver
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

    @time begin
    solution=solve!(prob, opts_altro); # solve with ALTRO
    end

#writing the solutions

	io = open("nom_tr.txt", "w")
    for i in prob.U
		for k=1:m-1
        	print(io,i[k])
        	print(io," ")
		end
		println(io,i[m])
    end
    close(io)


#Parsing the nominal trajectory to different arrays

	x=Array{Array{Float64,1}}(undef,agent_number)
	z=Array{Array{Float64,1}}(undef,agent_number)
	u1=Array{Array{Float64,1}}(undef,agent_number)
	u2=Array{Array{Float64,1}}(undef,agent_number)
	for i=1:agent_number
    	x[i] = [prob.X[k][3*i-2] for k = 1:N]
    	z[i] = [prob.X[k][3*i-1] for k = 1:N]
		u1[i]=[prob.U[k][2*i-1] for k = 1:N-1]
		u2[i]=[prob.U[k][2*i] for k = 1:N-1]
	end



#Plot and Animate

	a=plot(xlim=(-1,20),ylim=(-1,20))

	plot!(x[1],z[1],xlabel="x1",ylabel="x2",label="1",legend=:topleft,color=:blue,width=2,ratio=:equal,title="Multi-Agent Example Trajectories",alpha=0.5)
	plot!(x[2],z[2],label="2",legend=:topleft,width=2,ratio=:equal,alpha=0.5)
	plot!(x[3],z[3],label="3",legend=:topleft,width=2,ratio=:equal,alpha=0.5)
	plot!(x[4],z[4],label="4",legend=:topleft,width=2,ratio=:equal,alpha=0.5)
	plot!(x[5],z[5],label="5",legend=:topleft,width=2,ratio=:equal,alpha=0.5)
	plot!(x[6],z[6],label="6",legend=:topleft,width=2,ratio=:equal,alpha=0.5)


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

	plot!([-1.0,18.0],[0.0,0.0],color=:black,label="")
	plot!([-1.0,8.0],[2.4,2.4],color=:black,label="")
	plot!([12.0,18.0],[2.4,2.4],color=:black,label="")
	plot!([-1.0,8.0],[9.15,2.4],color=:black,label="")
	plot!([-1.0,12.0],[12.15,2.4],color=:black,label="")

	plot!([-1.0,18.0],[0.6,0.6],color=:red,label="")
	plot!([-1.0,9.8],[1.8,1.8],color=:red,label="")
	plot!([11.8,18.0],[1.8,1.8],color=:red,label="")
	plot!([-1.0,9.8],[9.9,1.8],color=:red,label="")
	plot!([-1.0,11.8],[11.4,1.8],color=:red,label="")


	anim = @animate for i in 1:N
		plot((x[1][i],z[1][i]),marker=:circle,xlim=(0,15),ylim=(0,15))
		plot!((x[2][i],z[2][i]),marker=:circle)
		plot!((x[3][i],z[3][i]),marker=:circle)
		plot!((x[4][i],z[4][i]),marker=:circle)
		plot!((x[5][i],z[5][i]),marker=:circle)
		plot!((x[6][i],z[6][i]),marker=:circle)

		plot!([0.0,18.0],[0.0,0.0],color=:black,label="")
		plot!([0.0,8.0],[2.4,2.4],color=:black,label="")
		plot!([12.0,18.0],[2.4,2.4],color=:black,label="")
		plot!([0.0,8.0],[8.4,2.4],color=:black,label="")
		plot!([0.0,12.0],[11.4,2.4],color=:black,label="")

		plot!([0.0,18.0],[0.6,0.6],color=:red,label="")
		plot!([0.0,9.8],[1.8,1.8],color=:red,label="")
		plot!([11.8,18.0],[1.8,1.8],color=:red,label="")
		plot!([0.0,9.8],[9.15,1.8],color=:red,label="")
		plot!([0.0,11.8],[10.65,1.8],color=:red,label="")
	end
