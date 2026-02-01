function [Aa,Ba,Cza,Dzua,Bwa,Dwa] = AugmentedState(A,B)
    [nx,~] = size(B);
    rho = 0.001;
    Cz = [0 1]*rho; 
    Dzu = sqrt(rho);
    
    Cr = [0 1];
    Dr = 0;
    Aa = [0 0 -Cr;1 0 zeros(1,nx);zeros(nx,1) zeros(nx,1) A];
    Ba = [-Dr;0;B];
    Bw = 0.1*B;
    Fr = 0;
    Fz = 0;
    Bwa = [-Fr 1;0 0;Bw zeros(2,1)];
    Dwa = [Fz 0];
    Cza = [0 0 Cz];
    Dzua = Dzu;
end

