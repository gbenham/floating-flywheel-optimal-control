# floating_flywheel_example.jl
# Annotated and cleaned version of the uploaded script (sinusoid_price_example.jl).
#
# Purpose:
#  - Direct transcription of a simple optimal-control example:
#      dω/dt = T(t) - ω^2(t)
#    with periodic boundary ω(0)=ω(T), and bounds -1 ≤ T ≤ 1.
#  - Revenue defined as R = ∫ -c(t)*T(t)*ω(t) dt
#  - The NEGATIVE objective is explicitly MAXIMISED (JuMP Max sense).
#
# Usage:
#  julia --project=. floating_flywheel_example.jl
#
using JuMP
using Ipopt
using Plots
using Statistics
using DelimitedFiles

# ---------------------------
# Parameters
# ---------------------------
c1 = 0.5           # price contrast parameter
tf = 1.0           # period (dimensionless)
N = 1000           # number of grid points
t = range(0.0, stop=tf, length=N)
dt = t[2] - t[1]

# Sinusoidal price profile
c = (1 + c1)/2 .+ 0.5*(1 - c1).*cos.(2*pi.*t./tf)

# ---------------------------
# Model and solver
# ---------------------------
model = Model(Ipopt.Optimizer)
set_optimizer_attribute(model, "tol", 1e-8)
set_optimizer_attribute(model, "max_iter", 2000)
set_optimizer_attribute(model, "print_level", 0)

@variable(model, ω[1:N] >= 0.0, start = 0.2)
@variable(model, -1.0 <= T[1:N] <= 1.0, start = 0.0)

@constraint(model, ω[1] == ω[N])
@constraint(model, T[1] == T[N])

# ---------------------------
# Dynamics
# ---------------------------
@NLconstraint(model, [i=2:N-1],
    (ω[i+1] - ω[i-1]) / (2*dt) == T[i] - ω[i]^2)
@NLconstraint(model, (ω[2] - ω[1]) / dt == T[1] - ω[1]^2)
@NLconstraint(model, (ω[N] - ω[N-1]) / dt == T[N] - ω[N]^2)

# ---------------------------
# Objective: MAXIMISE revenue
# R = ∫ -c(t)*T(t)*ω(t) dt  (trapezoidal rule)
# ---------------------------
@NLexpression(model, integrand[i=1:N], -c[i] * T[i] * ω[i])
@NLexpression(model, revenue_trap,
    0.5*dt*(integrand[1] + integrand[N] + 2*sum(integrand[i] for i=2:N-1)))
@NLobjective(model, Max, revenue_trap)

# ---------------------------
# Solve
# ---------------------------
optimize!(model)

ω_opt = value.(ω)
T_opt = value.(T)

println("Objective (revenue) = ", objective_value(model))

# ---------------------------
# Diagnostics
# ---------------------------
residuals = zeros(N)
for i in 2:(N-1)
    residuals[i] = abs((ω_opt[i+1] - ω_opt[i-1])/(2*dt) - (T_opt[i] - ω_opt[i]^2))
end
residuals[1] = abs((ω_opt[2] - ω_opt[1])/dt - (T_opt[1] - ω_opt[1]^2))
residuals[N] = abs((ω_opt[N] - ω_opt[N-1])/dt - (T_opt[N] - ω_opt[N]^2))

println("Max dynamics residual = ", maximum(residuals))

# ---------------------------
# Save outputs
# ---------------------------
plot(t, ω_opt, label="ω(t)")
plot!(t, T_opt, label="T(t)")
plot!(t, c, label="c(t)", linestyle=:dash)
xlabel!("t (dimensionless)")
ylabel!("dimensionless variables")
savefig("solution_plot.png")

writedlm("results_traj.csv", hcat(collect(t), ω_opt, T_opt, c), ',')
