function v = quaternionRotate (q, vec)
   % This function will rotate the vectors v (1x3 or Nx3) by the quaternions
   % q (1x4 or Nx4)
   % Result: q * [0,v] * q'
   % The result will always be a vector (Nx3)
   
   qInv = quaternionInvert(q);
   qv = quaternionMultiply(quaternionMultiply(q, [zeros(size(vec, 1), 1), vec]), qInv);
   v = qv(:, 2:4);
end
