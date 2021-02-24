using TrajectoryOptimization
using Plots, LinearAlgebra




#=
io_read = open("/home/mehrdad/Scots+Altro/examples/unicycle1D/T_uni.txt", "r")
aa=readlines(io_read)
FF=map(x->parse(Float32,x),aa)
io_read2 = open("/home/mehrdad/Scots+Altro/examples/cart_pole_MA/T_cp.txt", "r")
aa2=readlines(io_read2)
FF2=map(x2->parse(Float32,x2),aa2)

io_read3 = open("/home/mehrdad/Scots+Altro/examples/cart_pole_MA/T_cpc.txt", "r")
aa3=readlines(io_read3)
FF3=map(x3->parse(Float64,x3),aa3)

io_read4 = open("/home/mehrdad/Scots+Altro/examples/unicycle1D/T_uniC.txt", "r")
aa4=readlines(io_read4)
FF4=map(x4->parse(Float64,x4),aa4)
=#


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

n=6
m=2

model = Model(cart_pole!,n,m)


#model = Dynamics.cartpole_urdf
model_d = rk4(model)
n = model.n;
m = model.m;

T = Float64;

x0 = [0.0;0.0;pi;0.0;8.0;0.0]
xf = [5.0;0.0;pi;0.0;4.0;0.0]

Q = 0.01*Diagonal(I,n)
Qf = 1000.0*Diagonal(I,n)
R = 0.01*Diagonal(I,m)

goal = goal_constraint(xf);

N = 70#plot(prob.U,xlabel="time step",title="Control Trajectory",label=["u1" "u2"])
dt = 0.1
tf = (N-1)*dt

U = [ones(m) for k = 1:N-1]

obj = LQRObjective(Q,R,Qf,xf,N)
constraints = Constraints(N)


u_min=[-6.0,-5.0]
u_max=[6.0,5.0]

bnd = BoundConstraint(n,m,u_min=u_min,u_max=u_max)

for k = 1:N-1
     constraints[k] += bnd
 end


function My_constraint(x)
	ϵ=0.35;
	l=0.5;
	X1=x[1]+l*sin(x[3]);
	Y1=0.6+l*cos(x[3]);
	X2=x[5]
	#Y2=0.25;
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


constraints[N] += goal

prob = Problem(model_d,obj,constraints=constraints,x0=x0,N=N,dt=dt,xf=xf)
#initial_controls!(prob, U);

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


#=
    io = open("/home/mehrdad/Scots+Altro/examples/Collision Test/cart_pole_ST.txt", "w")
    for i in prob.X
        print(io,i[1])
        print(io," ")
		print(io,i[2])
        print(io," ")
		print(io,i[3])
        print(io," ")
		print(io,i[4])
        print(io," ")
		print(io,i[5])
        print(io," ")
        println(io,i[6])
    end
    close(io)
=#
#=
	io = open("/home/mehrdad/Scots+Altro2/examples/cart_pole_MA/cart_pole_new.txt", "w")
	for i in prob.U
		print(io,i[1])
		print(io," ")
		println(io,i[2])
	end
	close(io)
=#
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
#=
	for i = 1:6*N
		Xd1[i]=xd1[i]+l*sin(xd3[i]);
		Yd1[i]=0.6+l*cos(xd3[i]);
		Y3[i]=0.6;
		Y2[i]=0.25;
		Xc1[i]=xc1[i]+l*sin(xc3[i]);
		Yc1[i]=0.6+l*cos(xc3[i]);
	end
	for i = 1:6*N
		#if(sqrt((X1[i]-X2[i])^2 + (Y1[i]-Y2[i])^2)<0.1)
			dist[i]=sqrt((Xc1[i]-Xc2[i])^2 + (Yc1[i]-Y2[i])^2)
		if(dist[i]>0.4)
			dist[i]=0.4
		end

		#else
	#		dist[i]=0.1;
	#	end
	end
=#

	#print("min "); println( minimum(x));
    #print("max ") ;println( maximum(b));


    #c=plot()

	c=plot(dist)
#	d=plot(prob.X,xlabel="time step",title="State Trajectory",legend=:topleft)
#	b=plot(prob.U,xlabel="time step",title="input Trajectory")

#    plot_obstacles(circles,:orange)
#	a=plot();
#	plot!(X1,Y1,xlabel="x",ylabel="y",label="cart_pole",legend=:topleft,width=2,ratio=:equal,title="state trajectory")
#	plot!(X2,Y2,xlabel="x",ylabel="y",label="unicycle",legend=:topleft,width=2,ratio=:equal,title="state trajectory")
	#plot!(X2,Y2,xlabel="x",ylabel="y",label="unicycle",legend=:topleft,width=2,ratio=:equal,title="state trajectory")

    #plot!((Xd[1],z[1]),marker=:circle,color=:red,label="start")
    #plot!((x[end],z[end]),marker=:circle,color=:green,label="goal")

#PP1=minimum(prob.U)
#PP2=maximum(prob.U)

rectangle(w, h, x, y) = Shape(x .+ [0,w,w,0], y .+ [0,0,h,h])


	anim = @animate for i in 1:N
plot((X1[i],Y1[i]),marker=:circle,label="pole position",xlim=(0,7),ylim=(0,7))
plot!((X2[i],Y2[i]),marker=:circle,label="unicycle")

plot!(rectangle(0.6,0.5,X2[i]-0.3,0), opacity=.5)


plot!(x1[1:i],Y3[1:i],label="cart position")
plot!([x1[i],X1[i]],[Y3[i],Y1[i]])

	end

#println(prob.U[:,1])
#println(maximum(prob.U[:,2]))
#println(minimum(prob.U))
#println(minimum(prob.U[2,:]))
