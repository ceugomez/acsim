include("P1.jl")

function ConvertRadianToDegree(angle_in_radian)
    angle_in_degreees = angle_in_radian*180/pi
    return angle_in_degrees
end

function ConvertDegreeToRadian(angle_in_degree)
    angle_in_radians = angle_in_degree*pi/180
    return angle_in_radians
end

EulerAngles_in_degrees = [123, 10, -3]
EulerAngles_in_radians = broadcast(ConvertDegreeToRadian, EulerAngles_in_degrees)


#Part A)
VelocityInBodyFrame = [15,0,2]
WindAngles = AirRelativeVelocityVectorToWindAngles(VelocityInBodyFrame)

#Part B) and C)
WindInBodyFrame = [1,1,-1]
GroundVelocityInBodyFrame = VelocityInBodyFrame + WindInBodyFrame
GroundVelocityInInertialFrame = TransformFromBodyToInertial(GroundVelocityInBodyFrame, EulerAngles_in_radians)
