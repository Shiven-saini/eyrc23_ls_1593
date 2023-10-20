function K = lqr_Rotary_Inverted_Pendulum (A, B)
  C = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
  ## Initialise C matrix
  D = [0; 0; 0; 0];
  ## Initialise D matrix
  Q = eye(4);
  ## Initialise Q matrix
  R = 1;
  ## Initialise R 
  sys = ss(A, B, C, D);
  ## State Space Model
  K = lqr(sys, Q, R);
  ## Calculate K matrix from A,B,Q,R matrices using lqr()
endfunction

