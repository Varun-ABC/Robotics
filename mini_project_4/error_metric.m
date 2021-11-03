max_iter = 300;
alpha = .2;
el = .005;
w = 1;
for i = 1:2
    rng(5)
    for k = 1:25
        q = (rand(6,1)-.5)*pi;
        irb1200.q = q;
        irb1200 = fwddiffkiniter(irb1200);
        T = irb1200.T;
        irb1200.Weights=[w;w;w;1;1;1];
        irb1200.q = q + q .* .1 .* randn(6,1);
        if i ==1 
            irb1200=invkin_iterJ(irb1200, max_iter, alpha, el);
        end
        if i == 2
            w = 1;
            %el = .001;
            irb1200=invkin_iterJ_kth(irb1200, max_iter, alpha, el);
        end
        Tsol2=irb1200.T;
        er(i,k) = norm(T - Tsol2,'fro');
    end
end
er
