# Lib
using LinearAlgebra
using Plots
using Kronecker
using LaTeXStrings
using MAT

file1 = matopen("data1.mat");
file2 = matopen("data2.mat");
file3 = matopen("data3.mat");
file4 = matopen("data4.mat");
file5 = matopen("data5.mat");
dK_1 = read(file1,"dK");
dK_2 = read(file2,"dK");
dK_3 = read(file3,"dK");
dK_4 = read(file4,"dK");
dK_5 = read(file5,"dK");
n_learn = 80;


# Plot
# t=1:1:n_learn;
# plot(t[:,1], [dK_1 dK_2 dK_3 dK_4 dK_5], label=[L"λ = 0.5" L"λ = 0.7" L"λ = 0.9" L"PI" L"VI"],xlabel = "Iteration", linewidth=1)

t=collect(1:1:n_learn);
anim1 = @animate for i = 1:n_learn
    plot(t[1:i], [dK_1[1:i] dK_2[1:i] dK_3[1:i] dK_4[1:i] dK_5[1:i]], label=[L"λ = 0.5" L"λ = 0.7" L"λ = 0.9" L"PI" L"VI"],xlabel = "Iteration", linewidth=1)
end
gif(anim1, "Comparation.gif", fps = 3)

# anim2 = @animate for i = 1:n_learn
#     p4=plot(t[1:i],dKd1[1:i],label=L"|| K_{d_1} - K_{d_1}^*||",ylim=(-0.1, 0.6))
#     p5=plot(t[1:i],dKd2[1:i],label=L"|| K_{d_2} - K_{d_2}^*||",ylim=(-0.1, 0.5))
#     plot(p4, p5, layout=(2,1))
#     end
# gif(anim2, "WorstDisturbance.gif", fps = 5)