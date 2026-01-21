classdef Control
    
    properties
        A, B, Bw
        n, m, nw
        small_number
    end
    
    methods
        function obj = Control(A,B,Bw)
            obj.A = A;
            obj.B = B;
            obj.Bw = Bw;

            obj.n = size(A,1);
            obj.m = size(B,2);
            obj.nw = size(Bw, 2);

            obj.small_number = 1e-7;

        end
        
        function Psol = LMI_stability(obj)
            P = sdpvar(obj.n);
            
            C1 = P >= obj.small_number;
            
            
            C2 = (obj.A' * P + P * obj.A <= -obj.small_number * eye(obj.n));
            
            con = C1 + C2;
            opts = sdpsettings;
            opts.solver='sedumi';
            
            diag = optimize(con, [], opts);
            
            if diag.problem == 0
                disp('Stable System');
                Psol = double(P);
            else
                disp('Unstable System');
                Psol = []; 
            end
        end
        
        function K_inf = H_inf(obj,C11, D11, D12)
            Aa = obj.A;
            B1 = obj.B;
            B2 = obj.Bw;

            nn = obj.nw;
            nz = size(C11, 1);

            X = sdpvar(obj.n);
            Y = sdpvar(obj.m,obj.n);
            gamma = sdpvar();

            C1 = X >= obj.small_number;
            C2 = ([(Aa*X+B1*Y)+(Aa*X+B1*Y)'   B2          (C11*X+D12*Y)'; ...
                            B2'         -gamma*eye(nn)           D11'; ...
                       (C11*X+D12*Y)          D11           -gamma*eye(nz) ] <= -obj.small_number);
            C3 = gamma >= obj.small_number;
            con = C1+C2+C3;


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

            nn = obj.nw;
            nz = size(C33, 1);

            X = sdpvar(obj.n);
            Y = sdpvar(obj.m,obj.n);
            mu = sdpvar();
            zeta = sdpvar();

            C1 = X >= obj.small_number;
            C2 = mu >= obj.small_number;
            C3 = ([(Aa*X+B1*Y)'+(Aa*X+B1*Y)+lambda*X     B2; ...
                                B2'                       -mu*eye(nn) ]<= -obj.small_number);
            C4 = ([lambda*X         zeros(obj.n, nn)    (C33*X+D32*Y)'; ...
                 zeros(nn, obj.n)   (zeta-mu)*eye(nn)        D31'; ...
                  (C33*X+D32*Y)          D31                zeta*eye(nz)]>= obj.small_number);

            con = C1+C2+C3+C4;

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

            nn = obj.nw;
            nz = size(C22, 1);

            X = sdpvar(obj.n);
            Y = sdpvar(obj.m,obj.n);
            Q = sdpvar(nz);

            C1 = X >= obj.small_number;
            C2 = Q >= obj.small_number;
            C3 = ([(Aa*X+B1*Y)+(Aa*X+B1*Y)'     B2; ...
                             B2'                -eye(nn)]<= -obj.small_number);
            C4 = ([Q                (C22*X+D22*Y); ...
                   (C22*X+D22*Y)'           X    ]>= -obj.small_number);

            con = C1+C2+C3+C4;

            opts = sdpsettings;
            opts.solver='sedumi';   % or 'lmilab'
            
            optimize(con,trace(Q),opts);
            Xsol = double(X);
            Ysol = double(Y);

            K_2 = Ysol*inv(Xsol);
        end

        function K = Hinf_regional_stability(obj,C11,D11,D12,alpha,theta,r)
            Aa  = obj.A;
            B1 = obj.B;
            B2 = obj.Bw;
        
            nn  = obj.n;
            mm  = obj.m;
            nww = obj.nw;
            nz = size(C11,1);
        
            eps = obj.small_number;
        
            X     = sdpvar(nn,nn,'symmetric');
            Y     = sdpvar(mm,nn,'full');
            gamma = sdpvar(1);
        
            AclX = Aa*X + B1*Y;
        
            cth = cos(theta);
            sth = sin(theta);

            C1 = (X >= eps*eye(nn));
        
            C2 = ((Aa*X+B1*Y) + (Aa*X+B1*Y)' + 2*alpha*X<= -eps*eye(nn));
        
            C3 = ([ AclX + AclX'          B2              (C11*X + D12*Y)'; ...
                    B2'            -gamma*eye(nww)          D11'; ...
                    (C11*X + D12*Y)       D11         -gamma*eye(nz) ] <= -eps);
            
            C4 = ([ sth*(AclX + AclX')      cth*(AclX - AclX'); ...
                    cth*(AclX' - AclX)      sth*(AclX + AclX') ] <= -eps);        
            
            C5 = ([-r*X    AclX;
                    AclX'  -r*X] <= -eps);
            
            C6 = (gamma >= eps);
        
            Constraints = [C1, C2, C3, C4, C5, C6];
        
            opts = sdpsettings('solver','sedumi','verbose',1);
            optimize(Constraints, gamma, opts);
        
            Xsol = value(X);
            Ysol = value(Y);
        
            K = Ysol / Xsol;
        
        end

        function KHinfH2 = Hinf_H2_multiObjective(obj, C11, D12, D11, C22, D22, struct)
            Aa = obj.A;
            B1 = obj.B;
            B2 = obj.Bw;

            nn = obj.nw;
            
            nz1 = size(C11, 1); 
            nz2 = size(C22, 1); 

            X = sdpvar(obj.n);
            Y = sdpvar(obj.m,obj.n);
            gamma = sdpvar();
            Q = sdpvar(nz2);

            C1 = X >= obj.small_number;
            C2 = Q >= obj.small_number;
            C3 = ([(Aa*X + B1*Y) + (Aa*X + B1*Y)'             B2              (C11*X+D12*Y)'; ...
                            B2'                         -gamma*eye(nn)            D11'; ...
                    (C11*X+D12*Y)                             D11,             -gamma*eye(nz1)] <= -obj.small_number);
            C4 = ([(Aa*X + B1*Y) + (Aa*X + B1*Y)'            B2; ...
                        B2'                           -eye(nn)] <= -obj.small_number);
            C5 = ([Q            (C22*X+D22*Y); ...
                   (C22*X+D22*Y)'      X      ] >= obj.small_number);
            
            
            con = C1+C2+C3+C4+C5;

            opts = sdpsettings;
            opts.solver='sedumi';   % or 'lmilab'

            Ysol = 0;
            Xsol = 1;

            if isscalar(struct)
                a = struct;
                C6 = trace(Q)<= a-obj.small_number;
                con = con+C6;
                
                optimize(con,gamma,opts);
                Xsol = double(X);
                Ysol = double(Y);
            else 
                a = struct(1);
                b = struct(2);
                optimize(con,a*gamma+b*trace(Q));
                Xsol = double(X);
                Ysol = double(Y);
            end

            KHinfH2 = Ysol*inv(Xsol);
        end
    end
end

