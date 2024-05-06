function u = RtoEuler(R, sequence)
u = zeros([3 1]);

switch sequence
    case "313"
        u(1) = atan2(R(1,3), R(2,3));
        u(2) = acos(R(3,3));
        u(3) = atan2(R(3,1), -R(3,2));
    case "312"
        u(1) = atan2(R(1,2), R(2,2));
        u(2) = -asin(R(3,2));
        u(3) = atan2(R(3,1), R(3,3));
end