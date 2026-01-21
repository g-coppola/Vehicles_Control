classdef RobustControl    
    properties
        A, B, Bw 
        n, m, nw
        small_number
    end
    
    methods
        function obj = RobustControl(A,B,Bw)
            obj.A = A;
            obj.B = B;
            obj.Bw = Bw;

            obj.n = size(A{1},1);
            obj.m = size(B{1},2);
            obj.nw = size(Bw{1},2);
            obj.small_number = 1e-7;
        end
        
        function K_inf = H_infinity(obj,C11,D11,D12)
            Aa = obj.A;
            B1 = obj.B;
            B2 = obj.Bw;

            l = size(Aa,2);

            nn = obj.nw;
            nz = size(C11, 1);

            X = sdpvar(obj.n);
            Y = sdpvar(obj.m,obj.n);
            gamma = sdpvar();

            con = X >= obj.small_number;

            
            for i = 1:l
                C = ([(Aa{i}*X+B1{i}*Y)+(Aa{i}*X+B1{i}*Y)'   B2{i}          (C11*X+D12*Y)'; ...
                            B2{i}'         -gamma*eye(nn)           D11'; ...
                       (C11*X+D12*Y)          D11           -gamma*eye(nz) ] <= -obj.small_number);
                con = con + C;
            end

            C3 = gamma >= obj.small_number;
            con = con+C3;

            opts = sdpsettings;
            opts.solver='sedumi';   % or 'lmilab'
            
            optimize(con,gamma,opts);
            Xsol = double(X);
            Ysol = double(Y);

            K_inf = Ysol*inv(Xsol);
        end

        function K_1 = L_1(obj,C33, D31, D32, lambda)
            Aa = obj.A;
            B1 = obj.B;
            B2 = obj.Bw;

            l = size(Aa,2);

            nn = obj.nw;
            nz = size(C33, 1);

            X = sdpvar(obj.n);
            Y = sdpvar(obj.m,obj.n);
            mu = sdpvar();
            zeta = sdpvar();

             

            C1 = X >= obj.small_number;
            C2 = mu >= obj.small_number;

            con = C1+C2;

            for i = 1:l
                C = ([(Aa{i}*X+B1{i}*Y)'+(Aa{i}*X+B1{i}*Y)+lambda*X     B2{i}; ...
                                B2{i}'                       -mu*eye(nn) ]<= -obj.small_number);

                con = con + C;
            end

            C3 = ([lambda*X         zeros(obj.n, nn)    (C33*X+D32*Y)'; ...
                 zeros(nn, obj.n)   (zeta-mu)*eye(nn)        D31'; ...
                  (C33*X+D32*Y)          D31                zeta*eye(nz)]>= obj.small_number);

            con = con +  C3;

            opts = sdpsettings;
            opts.solver='sedumi';   % or 'lmilab'
            
            optimize(con,zeta,opts);
            Xsol = double(X);
            Ysol = double(Y);

            K_1 = Ysol*inv(Xsol);
        end

        function K_2 = H_2(obj,C22,D22)
            Aa = obj.A;
            B1 = obj.B;
            B2 = obj.Bw;

            l = size(Aa,2);

            nn = obj.nw;
            nz = size(C22, 1);

            X = sdpvar(obj.n);
            Y = sdpvar(obj.m,obj.n);
            Q = sdpvar(nz);

            C1 = X >= obj.small_number;
            C2 = Q >= obj.small_number;

            con = C1+C2;
            
            for i = 1:l
                C = ([(Aa{i}*X+B1{i}*Y)+(Aa{i}*X+B1{i}*Y)'     B2{i}; ...
                             B2{i}'                -eye(nn)]<= -obj.small_number);

                con = con + C ;
            end

            C3 = ([Q                (C22*X+D22*Y); ...
                   (C22*X+D22*Y)'           X    ]>= -obj.small_number);

            con = con + C3;
            opts = sdpsettings;
            opts.solver='sedumi';   % or 'lmilab'
            
            optimize(con,trace(Q),opts);
            Xsol = double(X);
            Ysol = double(Y);

            K_2 = Ysol*inv(Xsol);
        end
    end
end

