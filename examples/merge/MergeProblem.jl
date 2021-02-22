
using TrajectoryOptimization
using Plots, LinearAlgebra




agent_number=6;

function unicycle!(ẋ::AbstractVector{T},x::AbstractVector{T},u::AbstractVector{T}) where T
	#ϵ=1
	#if( (x[1]<x[4]+ϵ && x[1]>x[4]-ϵ) && (x[2]<x[5]+ϵ && x[2]>x[5]-ϵ) )
	#	x[1]=-2; x[2]=-2;x[4]=-2;x[5]=-2
	#end
	for k=1:agent_number
		ẋ[3*k-2] = u[2*k-1]*cos(x[3*k]);
		ẋ[3*k-1] = u[2*k-1]*sin(x[3*k]);
	    ẋ[3*k] = u[2*k];
	end
end



n=agent_number*3
m=agent_number*2

model = Model(unicycle!,n,m)


#model = Dynamics.cartpole_urdf
model_d = rk4(model)
n = model.n;
m = model.m;

T = Float64;

x0 = [
6.0;1.2;0.0;
3.0;1.2;0.0;
0.0;1.2;0.0;
8;3.9;-0.64;
6;5.4;-0.64;
4.0;6.9;-0.64
]

xf = [
17.0;1.2;0.0;
17.0;1.2;0.0;
17.0;1.2;0.0;
17.0;1.2;0.0;
17.0;1.2;0.0;
17.0;1.2;0.0
]

goal = goal_constraint(xf);

Q = 0.01*Diagonal(I,n)
Qf = 1000.0*Diagonal(I,n)
R = 0.01*Diagonal(I,m)



N = 110
dt = 0.1
tf = (N-1)*dt


U = [ones(m) for k = 1:N-1]

obj = LQRObjective(Q,R,Qf,xf,N)
constraints = Constraints(N)
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

function My_obs(c,x,u)
    c[1]=collision_constraint(x);
    c[2]=constraint1(x);
	c[3]=constraint2(x);
	c[4]=constraint3(x);
	return nothing
end
obs= Constraint{Inequality}(My_obs,n,m,4,:obs)

for k = 1:N-1
     constraints[k] += obs
end



#=
function goal1(v,x,u)
	v[1:3]=x[1:3]-[15.0;1.0;0.0]
	return nothing;
end
con_goal1=Constraint{Equality}(goal1, n, m, 3, :mygolcon1)

function goal2(v,x,u)
	v[1:3]=x[4:6]-[15.0;1.0;0.0]
	return nothing;
end
con_goal2=Constraint{Equality}(goal2, n, m, 3, :mygolcon2)

function goal3(v,x,u)
	v[1:3]=x[7:9]-[15.0;1.0;0.0]
	return nothing;
end
con_goal3=Constraint{Equality}(goal3, n, m, 3, :mygolcon3)

function goal4(v,x,u)
	v[1:3]=x[10:12]-[15.0;1.0;0.0]9.15
	return nothing;
end
con_goal4=Constraint{Equality}(goal4, n, m, 3, :mygolcon4)

function goal5(v,x,u)
	v[1:3]=x[13:15]-[15.0;1.0;0.0]
	return nothing;
end
con_goal5=Constraint{Equality}(goal5, n, m, 3, :mygolcon5)

function goal6(v,x,u)
	v[1:3]=x[16:18]-[15.0;1.0;0.0]
	return nothing;
end
con_goal6=Constraint{Equality}(goal6, n, m, 3, :mygolcon6)

constraints[floor(Int64,N*4/10)] += con_goal1
constraints[floor(Int64,N*6/10)] += con_goal2
constraints[floor(Int64,N*8/10)] += con_goal3
constraints[floor(Int64,N*5/10)] += con_goal4
constraints[floor(Int64,N*7/10)] += con_goal5
constraints[floor(Int64,N*9/10)] += con_goal6
=#


#obs = Constraint{Inequality}(circle_obs,n,m,n_circles,:obs);

 #bound=BoundConstraint(n, m,u_min=u_min)


#constraints[N] += goal

prob = Problem(model_d,obj,constraints=constraints,x0=x0,N=N,dt=dt,xf=xf)
#initial_controls!(prob, U);

# options
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
#=	io = open("/home/mehrdad/github/abcd-with-reference-controller/examples/merging_problem/tr.txt", "w")
    for i in prob.X
		for k=1:n-1
        	print(io,i[k])
        	print(io," ")
		end
		println(io,i[n])
    end
    close(io)=#
#=
	io = open("/home/mehrdad/Scots+Altro/examples/merge/tr_U.txt", "w")
    for i in prob.U
		for k=1:m-1
        	print(io,i[k])
        	print(io," ")
		end
		println(io,i[m])
    end
    close(io)
=#



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





	a=plot(xlim=(-1,20),ylim=(-1,20))


#=
		function circle_shape(x,y,r)x[3*i-2]-10.5
			θ=LinRange(0,2*π,500);
			x .+ r*sin.(θ) , y.+ r*cos.(θ)
		end
		plot!(circle_shape(6,6,2),seriestype=[:shape,],lw=0.2,
		c=:yellow,linecolor=:black,label="",fillalpha=0.2,aspect_ratio=1)
		plot!(circle_shape(6,6,1.6),seriestype=[:shape,],lw=0.9,
		c=:yellow,linecolor=:black,label="",fillalpha=0.2,aspect_ratio=1)
=#
	#plot_obstacles(circles,:orange,opacity=.5)


	#rectangle(w, h, x, y) = Shape(x .+ [0,w,w,0], y .+ [0,0,h,h])
	#plot(rectangle(1,1,1.2,1.2), opacity=.5)

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
		plot((x[1][i],z[1][i]),marker=:circle,xlim=(0,20),ylim=(0,20))
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
	io = open("/home/mehrdad/work/Merge_nom.txt", "w")
	for i in prob.X
		for k=1:n-1
	    	print(io,i[k])
	    	print(io," ")
		end
	    println(io,i[n])
	end
	close(io)
