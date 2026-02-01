function K_2 = H_2(A,B1,B2,Cz,Dzu)
            n = size(A,1);
            m = size(B1,2);
            nw = size(B2,2);
            nz = size(Cz, 1);

            X = sdpvar(n);
            Y = sdpvar(m,n);
            Q = sdpvar(nz);

            C1 = X >= 1e-7;
            C2 = Q >= 1e-7;

            C3 = ([(A*X+B1*Y)+(A*X+B1*Y)'     B2; ...
                             B2'                -eye(nw)]<= -1e-7);

            C4 = ([Q                (Cz*X+Dzu*Y); ...
                   (Cz*X+Dzu*Y)'           X    ]>= 1e-7);

            con = C1+C2+C3+C4;

            opts = sdpsettings;
            opts.solver='sedumi';   % or 'lmilab'
            
            optimize(con,trace(Q),opts);
            Xsol = double(X);
            Ysol = double(Y);

            K_2 = Ysol*inv(Xsol);
        end