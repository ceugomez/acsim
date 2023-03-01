#=
Author: Himanshu Gupta
Date: 26th January, 2023
=#

#Function to get rotation matrix that rotates around the z axis
function GetRotationMatrixFromInertialToV1(yaw)
    rotation_matrix = [
                        cos(yaw) sin(yaw) 0;
                        -sin(yaw) cos(yaw) 0;
                        0 0 1
                        ]
    return rotation_matrix
end

#Function to get rotation matrix that rotates around the y axis
function GetRotationMatrixFromV1ToV2(pitch)
    rotation_matrix = [
                        cos(pitch) 0 -sin(pitch);
                        0 1 0;
                        sin(pitch) 0 cos(pitch)
                        ]
    return rotation_matrix
end

#Function to get rotation matrix that rotates around the x axis
function GetRotationMatrixFromV2ToBody(roll)
    rotation_matrix = [
                        1 0 0;
                        0 cos(roll) sin(roll);
                        0 -sin(roll) cos(roll)
                        ]
    return rotation_matrix
end

#Function to get rotation matrix that rotates around z,y and then x axis
function RotationMatrix321(EulerAngles)

    #GetRotationMatrixToTransformFromInertialToBody
    yaw = EulerAngles[1]
    pitch = EulerAngles[2]
    roll = EulerAngles[3]

    RMFromIntertialToV1 = GetRotationMatrixFromInertialToV1(yaw)
    RMFromV1ToV2 = GetRotationMatrixFromV1ToV2(pitch)
    RMFromV2ToBody = GetRotationMatrixFromV2ToBody(roll)

    RM = RMFromV2ToBody*RMFromV1ToV2*RMFromIntertialToV1
    return RM
end

#Function to transform vector from body frame to inertial frame
function TransformFromBodyToInertial(VectorBody, EulerAngles)
    rotation_matrix = RotationMatrix321(EulerAngles)
    VectorBody_Matrix = reshape(VectorBody, length(VectorBody), 1)
    v = transpose(rotation_matrix)*VectorBody_Matrix
    return v
end

#Function to transform vector from inertial frame to body frame
function TransformFromInertialToBody(VectorBody, EulerAngles)
    rotation_matrix = RotationMatrix321(EulerAngles)
    VectorBody_Matrix = reshape(VectorBody, length(VectorBody), 1)
    v = rotation_matrix*VectorBody_Matrix
    return v
end

#Function to get Air Relative Vehcile Velocity Vector in Body Frame from Wind Angles
function WindAnglesToAirRelativeVelocityVector(WindAngles)
    V_A = WindAngles[1]
    beta = WindAngles[2]
    alpha = WindAngles[3]

    u = V_A*cos(alpha)*cos(beta)
    v = V_A*sin(beta)
    w = V_A*sin(alpha)*cos(beta)

    AirSpeedInBodyFrame =  reshape([u,v,w],3,1)  #This is the VelocityBody vector
    return AirSpeedInBodyFrame
end

#Function to get Wind Angles from Air Relative Vehcile Velocity Vector in Body Frame 
function AirRelativeVelocityVectorToWindAngles(VelocityBody)
    u = VelocityBody[1]
    v = VelocityBody[2]
    w = VelocityBody[3]

    V_A = sqrt( (u*u) + (v*v) + (w*w) )
    beta = asin(v/V_A)
    alpha = atan(w/u)

    WindAngles = [V_A, beta, alpha]
    return WindAngles
end
