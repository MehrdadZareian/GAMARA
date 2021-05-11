
import Pkg; Pkg.activate(@__DIR__); Pkg.instantiate();
#Write Altro folder instead of @__DIR__

using TrajectoryOptimization
using Plots, LinearAlgebra

#number individual systems
agent_number=5;
#Relative position of drones to each other
rel_pos = ((0.0,0.0),(0.0,1.5),(1.5,0.0),(0.0,-1.5),(-1.5,0.0),(-1.5,-1.5),(-1.5,1.5),(1.5,-1.5),(1.5,1.5))
#Intital position of center one
initial_pos=(0,0,pi/4);
#Goal position of center one
goal_pos=(15,15,pi/4);

#defining product system dynamics
function dynamics!(ẋ::AbstractVector{T},x::AbstractVector{T},u::AbstractVector{T}) where T
	for k=1:agent_number
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
x0 = Array{Float64}(undef,3*agent_number)
#goal point for product system
xf = Array{Float64}(undef,3*agent_number)

for i=1:agent_number
	x0[3*i-2]= initial_pos[1] + rel_pos[i][1];
	x0[3*i-1]= initial_pos[2] + rel_pos[i][2];
	x0[3*i]=initial_pos[3];

	xf[3*i-2]= goal_pos[1]+ rel_pos[i][1];
	xf[3*i-1]=  goal_pos[2]+ rel_pos[i][2];
	xf[3*i]= goal_pos[3];
end






#these 2 parameters are very important
#number of points in trajectory(number of time samples)
N = 100
#sampling time
dt = 0.1
#total time
tf = (N-1)*dt


#LQR Cost Matrices (same as default)
Q = 0.01*Diagonal(I,n)
Qf = 1000.0*Diagonal(I,n)
R = 0.01*Diagonal(I,m)
obj = LQRObjective(Q,R,Qf,xf,N)



constraints = Constraints(N)

#goal Constraint
goal = goal_constraint(xf);
constraints[N] += goal

#creating upperbound and lower bound for control input with bound constraint
u_min = Array{Float64}(undef,m)
u_max = Array{Float64}(undef,m)
u_c=4;
for k=1:m
	u_min[k]=-u_c;
	u_max[k]=u_c;
end
bnd = BoundConstraint(n,m,u_min=u_min,u_max=u_max)
for k = 1:N-1
     constraints[k] += bnd
 end


#constraint for obstacles
#obstacles parameters (Xposition,Yposition,Radius)
# radius(obstacle size) is 2*epsilon+ delta
obstacles=((7,11.5,2.9),(7,3.5,2.9))

#=
in this function we calculate infinite norm distance between
each pont and center of obstacle, then we select the closest
(with minimum distance) and tehn we calculate constraint penalty
 value for constraint.
=#
function obs_constriant1(x)
	M=10000
	temp=0
    	for i=1:agent_number
        	temp=max((x[3*i-2]-obstacles[1][1])^2,(x[3*i-1]-obstacles[1][2])^2);
			M=min(temp,M)
		end
    return -(M-obstacles[1][3]^2);
end
function obs_constriant2(x)
	M=10000
	temp=0
    	for i=1:agent_number
        	temp=max((x[3*i-2]-obstacles[2][1])^2,(x[3*i-1]-obstacles[2][2])^2);
			M=min(temp,M)
		end
    return -(M-obstacles[2][3]^2);
end



function AllConstraints(c,x,u)
    c[1]= obs_constriant1(x);
    c[2]= obs_constriant2(x);
	return nothing
end

obs= Constraint{Inequality}(AllConstraints,n,m,2,:obs)


for k = 1:N-1
     constraints[k] += obs
 end

 #=
 in this function we calculate infinite norm distance between
 each pair of agents, then we select the closest pair
 (with minimum distance) and then we calculate constraint penalty
  value for constraint.
 =#
 function formation_constraint(v,x,u)
	 index=0
	 for i=1:agent_number
 		for j=i+1:agent_number
			#cuurent distance
			distancesq1= (x[3*i-2]-x[3*j-2])^2 + (x[3*i-1]-x[3*j-1])^2
			#desired distance
			distancesq2=(rel_pos[i][1]-rel_pos[j][1])^2+ (rel_pos[i][2]-rel_pos[j][2])^2
			index=index+1;
			v[index]=distancesq2-distancesq1
		end
	end
 	return nothing;
 end
 form_con=Constraint{Equality}(formation_constraint, n, m, 10, :mygolcon1)

 for k = 1:N-1
      constraints[k] +=form_con
  end






prob = Problem(model_d,obj,constraints=constraints,x0=x0,N=N,dt=dt,xf=xf)


# options for ALTRO solver (same as default)
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
    a=solve!(prob, opts_altro); # solve with ALTRO
    end

#parsing the solution to different arrays
	x=Array{Array{Float64,1}}(undef,12)
	z=Array{Array{Float64,1}}(undef,12)
	for i=1:agent_number
    	x[i] = [prob.X[k][3*i-2] for k = 1:N]
    	z[i] = [prob.X[k][3*i-1] for k = 1:N]
	end



	#writing to file
	io = open("nom_tr.txt", "w")
	for i in prob.U
		for k=1:m-1
	    	print(io,i[k])
	    	print(io," ")
		end
	    println(io,i[m])
	end
	close(io)
println("Trajectory is written in nom_tr.txt");





a=plot()
rectangle(w, h, x, y) = Shape(x .+ [0,w,w,0], y .+ [0,0,h,h])
plot!(rectangle(2*obstacles[1][3],2*obstacles[1][3],obstacles[1][1]-obstacles[1][3],obstacles[1][2]-obstacles[1][3]), opacity=.5)
plot!(rectangle(2*obstacles[2][3],2*obstacles[2][3],obstacles[2][1]-obstacles[2][3],obstacles[2][2]-obstacles[2][3]), opacity=.5)





	plot!(x[1],z[1],xlabel="x",ylabel="y",label="1",legend=:topleft,color=:blue,width=2,ratio=:equal,title="Formation Example Trajectories")
	plot!(x[2],z[2],xlabel="x",ylabel="y",label="2",legend=:topleft,width=2,ratio=:equal)
	plot!(x[3],z[3],xlabel="x",ylabel="y",label="3",legend=:topleft,width=2,ratio=:equal)
	plot!(x[4],z[4],xlabel="x",ylabel="y",label="4",legend=:topleft,width=2,ratio=:equal)
	plot!(x[5],z[5],xlabel="x",ylabel="y",label="5",legend=:topleft,width=2,ratio=:equal)


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

plot!((x[1][5],z[1][5]),marker=:circle,color=:green,label="")
plot!((x[2][5],z[2][5]),marker=:circle,color=:green,label="")
plot!((x[3][5],z[3][5]),marker=:circle,color=:green,label="")
plot!((x[4][5],z[4][5]),marker=:circle,color=:green,label="")
plot!((x[5][5],z[5][5]),marker=:circle,color=:green,label="")

plot!((x[1][10],z[1][10]),marker=:circle,color=:blue,label="")
plot!((x[2][10],z[2][10]),marker=:circle,color=:blue,label="")
plot!((x[3][10],z[3][10]),marker=:circle,color=:blue,label="")
plot!((x[4][10],z[4][10]),marker=:circle,color=:blue,label="")
plot!((x[5][10],z[5][10]),marker=:circle,color=:blue,label="")

plot!((x[1][20],z[1][20]),marker=:circle,color=:green,label="")
plot!((x[2][20],z[2][20]),marker=:circle,color=:green,label="")
plot!((x[3][20],z[3][20]),marker=:circle,color=:green,label="")
plot!((x[4][20],z[4][20]),marker=:circle,color=:green,label="")
plot!((x[5][20],z[5][20]),marker=:circle,color=:green,label="")

plot!((x[1][30],z[1][30]),marker=:circle,color=:red,label="")
plot!((x[2][30],z[2][30]),marker=:circle,color=:red,label="")
plot!((x[3][30],z[3][30]),marker=:circle,color=:red,label="")
plot!((x[4][30],z[4][30]),marker=:circle,color=:red,label="")
plot!((x[5][30],z[5][30]),marker=:circle,color=:red,label="")

plot!((x[1][40],z[1][40]),marker=:circle,color=:blue,label="")
plot!((x[2][40],z[2][40]),marker=:circle,color=:blue,label="")
plot!((x[3][40],z[3][40]),marker=:circle,color=:blue,label="")
plot!((x[4][40],z[4][40]),marker=:circle,color=:blue,label="")
plot!((x[5][40],z[5][40]),marker=:circle,color=:blue,label="")

plot!((x[1][50],z[1][50]),marker=:circle,color=:green,label="")
plot!((x[2][50],z[2][50]),marker=:circle,color=:green,label="")
plot!((x[3][50],z[3][50]),marker=:circle,color=:green,label="")
plot!((x[4][50],z[4][50]),marker=:circle,color=:green,label="")
plot!((x[5][50],z[5][50]),marker=:circle,color=:green,label="")
