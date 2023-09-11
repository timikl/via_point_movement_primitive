function y = RBFp2Trj(RBF)
%% Trajektorijo q realiziram z DMP-ji
y = [];
S.x = 1;
while (S.x >=  exp(-RBF.a_x))
   [S]=RBFp_Integrate(RBF,S);
   y = [y;S.y];
end 