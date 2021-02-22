
using TrajectoryOptimization
using Plots, LinearAlgebra



#=
io_read = open("/home/mehrdad/Scots+Altro/examples/unicycle_MA/T1.txt", "r")
aa=readlines(io_read)
FF=map(x->parse(Float32,x),aa)
io_read2 = open("/home/mehrdad/Scots+Altro/examples/unicycle_MA/T2.txt", "r")
aa2=readlines(io_read2)
FF2=map(x2->parse(Float32,x2),aa2)
=#


function unicycle!(ẋ::AbstractVector{T},x::AbstractVector{T},u::AbstractVector{T}) where T
	for k=1:10
		ẋ[3*k-2] = u[2*k-1]*cos(x[3*k]);
		ẋ[3*k-1] = u[2*k-1]*sin(x[3*k]);
	    ẋ[3*k] = u[2*k];
	end
end



n=30
m=20

model = Model(unicycle!,n,m)


#model = Dynamics.cartpole_urdf
model_d = rk4(model)
n = model.n;
m = model.m;

T = Float64;

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

xf = [10.0;2.0;0.0;10.0;4.0;0.0;10.0;6.0;0.0;10.0;8.0;0.0;
2.0;10.0;pi/2;4.0;10.0;pi/2;6.0;10.0;pi/2;8.0;10.0;pi/2;
10.0;10.0;pi/4;10.0;0.0;-pi/4]
Q = 0.01*Diagonal(I,n)
Qf = 1000.0*Diagonal(I,n)
R = 0.01*Diagonal(I,m)

goal = goal_constraint(xf);

N = 102#plot(prob.U,xlabel="time step",title="Control Trajectory",label=["u1" "u2"])
dt = 0.1
tf = (N-1)*dt


U = [ones(m) for k = 1:N-1]

obj = LQRObjective(Q,R,Qf,xf,N)
constraints = Constraints(N)
u_min = Array{Float64}(undef,20)
u_max = Array{Float64}(undef,20)

u_c=2;
for k=1:20
	u_min[k]=-u_c;
	u_max[k]=u_c;
end


bnd = BoundConstraint(n,m,u_min=u_min,u_max=u_max)


for k = 1:N-1
     constraints[k] += bnd
 end


r_circle=2.0
circles = ((6.0,6.0,r_circle),(0.0,0.0,0.0))
n_circles = length(circles)





function My_rectangle(x)
	M=10000
	ϵ=1.46
		for i=1:10
        	temp=max((x[3*i-2]-5)^2,(x[3*i-1]-5)^2)
			M=min(temp,M)
    	end
    return -(M-ϵ^2);
end




function My_constraint(x)
	ϵ=0.64
	M=1000;
	for i=1:10
		for j=i+1:10
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

#obs = Constraint{Inequality}(circle_obs,n,m,n_circles,:obs);

 #bound=BoundConstraint(n, m,u_min=u_min)


for k = 1:N-1
     constraints[k] += obs
 end



constraints[N] += goal

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
    a=solve!(prob, opts_altro); # solve with ALTRO
    end

	Pnumber=4;
	#N=Pnumber;
    x = [prob.X[k][1] for k = 1:N]
    z = [prob.X[k][2] for k = 1:N]
	x2 = [prob.X[k][4] for k = 1:N]
    z2 = [prob.X[k][5] for k = 1:N]
	x3 = [prob.X[k][7] for k = 1:N]
    z3 = [prob.X[k][8] for k = 1:N]
	x4 = [prob.X[k][10] for k = 1:N]
    z4 = [prob.X[k][11] for k = 1:N]

	diff1=(x-x2).^2 + (z-z2).^2;
	diff2=(x-x3).^2 + (z-z3).^2;
	diff3=(x3-x2).^2 + (z3-z2).^2;

	diff4=(x4-x).^2 + (z4-z).^2;
	diff5=(x4-x2).^2 + (z4-z2).^2;
	diff6=(x4-x3).^2 + (z4-z3).^2;





	x=Array{Array{Float64,1}}(undef,12)
	z=Array{Array{Float64,1}}(undef,12)
	for i=1:10
    	x[i] = [prob.X[k][3*i-2] for k = 1:N]
    	z[i] = [prob.X[k][3*i-1] for k = 1:N]
	end
#=
		x[11]=FF[1:2:end]
		z[11]=FF[2:2:end]

		x[12]=FF2[1:2:end]
		z[12]=FF2[2:2:end]
=#
	a=plot()

rectangle(w, h, x, y) = Shape(x .+ [0,w,w,0], y .+ [0,0,h,h])
plot!(rectangle(2,2,4,4), opacity=.5)
plot!(rectangle(2.8,2.8,3.6,3.6), opacity=.3)


#=		function circle_shape(x,y,r)
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
#	plot!(x[11],z[11],xlabel="x",ylabel="y",label="1+D",legend=:topleft,width=2,ratio=:equal,color=:red)
#	plot!(x[12],z[12],xlabel="x",ylabel="y",label="2+D",legend=:topleft,width=2,ratio=:equal,color=:red)


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

#=
		M=Array{Float64,1}(undef,N);

		for k=1:N
			temp=1000;
			for i=1:10
				for j=i+1:10
					temp=min(temp,((x[i][k]-x[j][k])^2 +(z[i][k]-z[j][k])^2))
				end
			end
			M[k]=temp;
		end
		plot(M,title="minimum distance^2")


	#plot(rectangle(1,1,1.2,1.2), opacity=.5)
=#
#=
    plot!(x,z,xlabel="x",ylabel="y",label="first",legend=:topleft,color=:blue,width=2,ratio=:equal,title="state trajectory")
    plot!((x[1],z[1]),marker=:circle,color=:red,label="start")
    plot!((x[end],z[end]),marker=:circle,color=:green,label="goal")
	plot!(x2,z2,label="second",legend=:topleft,color=:green,width=2,ratio=:equal)
    plot!((x2[1],z2[1]),	marker=:circle,color=:red,label="start")
    plot!((x2[end],z2[end]),marker=:circle,color=:green,label="goal")
	plot!(x3,z3,label="Third",legend=:topleft,color=:red,width=2,ratio=:equal)
    plot!((x3[1],z3[1]),	marker=:circle,color=:red,label="start")
    plot!((x3[end],z3[end]),marker=:circle,color=:green,label="goal")
	plot!(x4,z4,label="4th",legend=:topleft,width=2,ratio=:equal)
    plot!((x4[1],z4[1]),	marker=:circle,color=:red,label="start")
    plot!((x4[end],z4[end]),marker=:circle,color=:green,label="goal")
=#
    #plot(prob.U,xlabel="time step",title="Input Trajectory")
	#plot(prob.X,xlabel="time step",title="State Trajectory")
#=
	plot(diff1,title="Distances^2")
	plot!(diff2)
	plot!(diff3)
	plot!(diff4)
	plot!(diff5)
	plot!(diff6)
=#
	#max_violation(a)

#plot(prob.U,xlabel="time step",title="Control Trajectory",label=["u1" "u2"])
#println(prob.U[:,1])
#println(maximum(prob.U[:,2]))
#println(minimum(prob.U))
#println(minimum(prob.U[2,:]))

#=
io = open("/home/mehrdad/work/unicycle_MA_final.txt", "w")
for i in prob.U
	for k=1:19
    	print(io,i[k])
    	print(io," ")
	end
    println(io,i[20])
end
close(io)
=#
#rectangle(w, h, x, y) = Shape(x .+ [0,w,w,0], y .+ [0,0,h,h])
