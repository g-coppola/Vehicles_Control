function K_inf = Hinfinity(A,B1,B2,Cz, Dw, Dzu)
            n = size(A,1);
            m = size(B1,2);
            nw = size(B2,2);
            nz = size(Cz, 1);

            X = sdpvar(n);
            Y = sdpvar(m,n);
            gamma = sdpvar();

            C1 = X >= 1e-7;
            C2 = ([(A*X+B1*Y)+(A*X+B1*Y)'   B2          (Cz*X+Dzu*Y)'; ...
                            B2'         -gamma*eye(nw)           Dw'; ...
                       (Cz*X+Dzu*Y)          Dw           -gamma*eye(nz) ] <= -1e-7);
            C3 = gamma >= 1e-7;
            con = C1+C2+C3;


            opts = sdpsettings;
            opts.solver='sedumi';   % or 'lmilab'
            
            optimize(con,gamma,opts);
            Xsol = double(X);
            Ysol = double(Y);

            K_inf = Ysol*inv(Xsol);
        end

