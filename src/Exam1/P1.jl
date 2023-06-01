#Definitions
x = AircraftState(
    100.0,
    200.0,
    -1655.0,
    -12*pi/180,
    9*pi/180,
    140*pi/180,
    15.0,
    -3.0,
    1.0,
    0.08*pi/180,
    -0.2*pi/180,
    0.0*pi/180
    )

wind_inertial = [0, 1, -2]

#P1
ϕ = x.ϕ
q = x.q
r = x.r
θ_dot = q*cos(ϕ) - r*sin(ϕ)

#P2
euler_angles = EulerAngles(x.ϕ, x.θ, x.ψ)
wind_body_frame = TransformFromInertialToBody(wind_inertial, euler_angles)
Va = x[7:9] - wind_body_frame   #Va vector in body frame
wind_angles = AirRelativeVelocityVectorToWindAngles(Va)
α = wind_angles.α

#P3
m = 10
w_dot = 0.05
θ = x.θ
u = x.u
p = x.p
v = x.p
g = 9.81
fZ = m*(w_dot -q*u + p*v)
fZ_gravity = m*g*cos(θ)*cos(ϕ)
fZ_aero = fZ - fZ_gravity
