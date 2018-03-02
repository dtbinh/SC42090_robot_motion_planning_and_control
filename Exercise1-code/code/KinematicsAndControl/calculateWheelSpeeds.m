function [ LeftWheelVelocity, RightWheelVelocity ] = calculateWheelSpeeds( vu, omega, parameters )
%CALCULATEWHEELSPEEDS This function computes the motor velocities for a differential driven robot

wheelRadius   = parameters.wheelRadius;
halfWheelbase = parameters.interWheelDistance/2;

WheelVelocity      = (2/wheelRadius)/[1,1;1,-1]*[vu;halfWheelbase*omega];
RightWheelVelocity = WheelVelocity(1);
LeftWheelVelocity  = WheelVelocity(2);
end
