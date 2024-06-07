# Code for paper: Model-Free λ-Policy Iteration for Discrete-Time
# Linear Quadratic Regulation

# Programing Language : Julia 
# Method : Off-Policy
# Purpose : Practice and Research

# Lib
using LinearAlgebra
using Plots
using Kronecker
using LaTeXStrings
using MAT
# Model
A = [0.9065 0.0816 -0.0005;
0.0743 0.9012 -0.0007;
0 0 0.1327];
B=[-0.0027,-0.0068,1][:,:];

# Value function parameters
Q = Matrix(Diagonal([1,1,1]));
R=[1][:,:];
λ = 0.5;

# Optimal Control
K_s = [0.0643 0.0699 -0.0667];
# 
n = size(A, 2);
m = size(B, 2);
add =20;
f = n^2 + m^2 + n * m + add;
n_learn = 80;
x0 = [1, -1, 1] # Initial state
K_0 = [0 -0.12 -1];
global i = 1;
K = [K_0];
P_0 = zeros(n,n);
X2_0 = zeros(m,m);
X3_0 = zeros(n,m);
P = [P_0];
X2=[X2_0];
X3=[X3_0]


# phi1 = []; phi2 = []; phi3 = []; phi4 = []; phi5 = []; phi6 = []; phi7 = [];
# phi8 = []; phi9 = []; phi10 = []; phi11 = []; phi12 = []; phi13 = []; phi14 = []; phi15 = [];
# phi16 = []; phi17 = []; phi18 = []; phi19 = []; phi20 = []; phi21 = [];
# phi = []; psi = [];

# Collect data to use Off-policy RL algorithm
x = zeros(n, f+1);
x[:, 1]=x0;
u = zeros(m, f);

for k in 1:f
    e = sin(0.5*k)^2 + sin(k) + cos(k);
    # probing noise
    u[:,k] = K[i] * x[:, k] + [e];
    x[:, k+1] = A * x[:, k] + B * u[:, k];
end

# Train
while true 
    global Phi = zeros(Float64, f, f-add);
    global Psi = zeros(Float64, f, 1);
    for k in 1:f
        global phi1 = kron(x[:, k]', x[:, k]') - λ*kron(x[:, k+1]', x[:, k+1]');
        global phi2 = λ * kron((u[:, k] - K[i] * x[:, k])', (u[:, k] + K[i] * x[:, k])');
        global phi3 = 2 * λ * kron((u[:, k] - K[i] * x[:, k])', x[:, k]');
        global psi = (x[:, k]') * (Q + K[i]' * R * K[i]) * x[:, k] +(1-λ)*(x[:, k+1]'*P[i]*x[:, k+1]-2*x[:, k]'*X3[i]*(u[:, k] - K[i] * x[:, k])-(u[:, k] + K[i] * x[:, k])'*X2[i]*(u[:, k] - K[i] * x[:, k]));
        global phi =hcat(phi1, phi2, phi3);
        # append!(Phi, phi);
        # append!(Psi, psi);
        Phi[k,:] = phi;
        Psi[k] = psi;
    end
    global vec_X = pinv(Phi' * Phi) * Phi' * Psi;
    X1_u = vec_X[1:n^2];
    X2_u = vec_X[n^2+1:n^2+m^2];
    X3_u = vec_X[n^2+m^2+1:end];

    X1_u = reshape(X1_u,(n,n));
    X2_u = reshape(X2_u,(m,m));
    X3_u = reshape(X3_u,(n,m));

    K_u = -pinv(R+X2_u)*X3_u';
    # Find Optimal Solution Step by Step
    push!(K, K_u);
    push!(P, X1_u);
    push!(X2, X2_u);
    push!(X3, X3_u);

    global i = i+1;
    if i>n_learn
        break
    end
end

# Calculate Error
dK = zeros(n_learn, 1);

for j in 1:n_learn
    dK[j] = norm(K[j] - K_s);
end

# Plot
t=1:1:n_learn;
# p1=plot(t[:,1],dK,label=L"|| K - K^*||",xlabel = "Iteration",legendfontsize=7,line=:solid, marker=:circle, color=:Set3_3,lw =2,markersize=4)
# plot(p1, p2, p3, layout=(3,1))

file = matopen("data1.mat", "w")
write(file, "dK", dK)
close(file)