function Rt = perturbDCM(Rs)

for i=1:3
    Rt(:,i) = Rs(:,i) + 0.01.*rand([3 1]);
    Rt(:,i) = Rt(:,i).'/norm(Rt(:,i));
end