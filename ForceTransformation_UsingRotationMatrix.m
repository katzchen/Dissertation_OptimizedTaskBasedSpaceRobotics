function F_final =ForceTransformation_UsingRotationMatrix(RotationMatrix,deltaXYZ_original,F_original) 
%step 1: rotate to the new frame
%step 2: move to new location


Distance = deltaXYZ_original'; %DistanceFinal_original Distance from original to final in original frame
Distance_skew = [0 -Distance(3) Distance(2);Distance(3) 0 -Distance(1);-Distance(2) Distance(1) 0];
Tfinal_original = ([RotationMatrix zeros(3);Distance_skew*RotationMatrix RotationMatrix]);

F_final = Tfinal_original*F_original;
