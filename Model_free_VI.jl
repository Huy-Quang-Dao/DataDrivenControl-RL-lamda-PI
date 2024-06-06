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
# λ = 0;

# Optimal Control
K_s = [0.0643 0.0699 -0.0667];
# 
n = size(A, 2);
m = size(B, 2);
q = m+n;
n_end = 40;
n_learn = 80;
x0 = [1, -1, 1] # Initial state
K_0 = [0 -0.12 -1];
global i = 1;
K = [K_0];
H_0 = zeros(q,q);
H = [H_0];


# Collect data to use Off-policy RL algorithm
x = zeros(n, n_end+1);
x[:, 1]=x0;
u = zeros(m, n_end);
z = zeros(q, n_end+1);
# for k in 1:f
#     e = sin(0.5*k)^2 + sin(k) + cos(k);
#     # probing noise
#     u[:,k] = K[i] * x[:, k] + [e];
#     x[:, k+1] = A * x[:, k] + B * u[:, k];
# end

# Train
while true 
    global Phi = zeros(Float64, n_end, q^2);
    global Psi = zeros(Float64, n_end, 1);
    for k in 1:n_end
        e = sin(0.5*k)^2 + sin(k) + cos(k);
        u[:,k] = K[i] * x[:, k] + [e];
        x[:, k+1] = A * x[:, k] + B * u[:, k];
        z[:, k] = vcat(x[:, k],u[:, k]);
        z[:, k+1] = vcat(x[:, k+1],K[i]*x[:, k+1]);
        global psi = (x[:, k]') * (Q) * x[:, k]+(u[:, k]') * (R) * u[:, k] +(z[:, k+1]'*H[i]*z[:, k+1]);
        global phi = kron(z[:, k]',z[:, k]');
        # append!(Phi, phi);
        # append!(Psi, psi);
        Phi[k,:] = phi;
        Psi[k] = psi;
    end
    global vec_X = pinv(Phi' * Phi) * Phi' * Psi;
    X = reshape(vec_X,(q,q))
    X1_u = vec_X[1:n^2];
    X2_u = vec_X[n^2+1:n^2+m*n];
    X3_u = vec_X[n^2+m*n+1:n^2+2*m*n];
    X4_u = vec_X[n^2+2*m*n+1:end];

    X1_u = reshape(X1_u,(n,n));
    X2_u = reshape(X2_u,(n,m));
    X3_u = reshape(X3_u,(m,n));
    X4_u = reshape(X4_u,(m,m));

    K_u = -pinv(X4_u)*X3_u;
    # Find Optimal Solution Step by Step
    push!(K, K_u);
    push!(H, X);


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

file = matopen("data5.mat", "w")
write(file, "dK", dK)
close(file)