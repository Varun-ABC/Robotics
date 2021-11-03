max_iter = 300;
alpha = .2;
el = .005;
w = 1;
i = 0;
for alpha = linspace(.05,.5,4)
    rng(5)
    i = i+1;
    for k = 1:25
        q = (rand(6,1)-.5)*pi;
        irb1200.q = q;
        irb1200 = fwddiffkiniter(irb1200);
        T = irb1200.T;
        
        %             irb1200.MaxIter = 100;
        %             irb1200.StepSize = .4;
        irb1200.Weights=[w;w;w;1;1;1];
        irb1200.q = q + q .* .1 .* randn(6,1);
        irb1200=invkin_iterJ(irb1200, max_iter, alpha, el);
        Tsol2=irb1200.T;
        del_alp(i,k) = norm(T - Tsol2,'fro');
    end
end

max_iter = 300;
alpha = .2;
el = .005;
w = 1;
i = 0;
for max_iter = linspace(100,1000,4)
    rng(5)
    i = i+1;
    for k = 1:25
        q = (rand(6,1)-.5)*pi;
        irb1200.q = q;
        irb1200 = fwddiffkiniter(irb1200);
        T = irb1200.T;
        
        %             irb1200.MaxIter = 100;
        %             irb1200.StepSize = .4;
        irb1200.Weights=[w;w;w;1;1;1];
        irb1200.q = q + q .* .1 .* randn(6,1);
        irb1200=invkin_iterJ(irb1200, max_iter, alpha, el);
        Tsol2=irb1200.T;
        del_MI(i,k) = norm(T - Tsol2,'fro');
    end
end

i = 0;
max_iter = 300;
alpha = .2;
el = .005;
w = 1;
for el = linspace(.001,.2,4)
    rng(5)
    i = i+1;
    for k = 1:25
        q = (rand(6,1)-.5)*pi;
        irb1200.q = q;
        irb1200 = fwddiffkiniter(irb1200);
        T = irb1200.T;
        
        %             irb1200.MaxIter = 100;
        %             irb1200.StepSize = .4;
        irb1200.Weights=[w;w;w;1;1;1];
        irb1200.q = q + q .* .1 .* randn(6,1);
        irb1200=invkin_iterJ(irb1200, max_iter, alpha, el);
        Tsol2=irb1200.T;
        del_el(i,k) = norm(T - Tsol2,'fro');
    end
end

i = 0;
max_iter = 300;
alpha = .2;
el = .005;
w = 1;
for w = linspace(.1,2,4)
    rng(5)
    i = i+1;
    for k = 1:25
        q = (rand(6,1)-.5)*pi;
        irb1200.q = q;
        irb1200 = fwdkiniter(irb1200);
        T = irb1200.T;
        irb1200.Weights=[w;w;w;1;1;1];
        irb1200.q = q + q .* .1 .* randn(6,1);
        irb1200=invkin_iterJ(irb1200, max_iter, alpha, el);
        Tsol2=irb1200.T;
        del_w(i,k) = norm(T - Tsol2,'fro');
    end
end
%%
for j = 1:4
    figure(1)
    plot(1:25, del_w(j,:), 'LineWidth',1.5)
    title("Change in Weights")
    xlabel("Run")
    ylabel('Error (m)')
    hold on 
    
    figure(2)
    plot(1:25, del_alp(j,:), 'LineWidth',1.5)
    title("Change in Step Size")
    xlabel("Run")
    ylabel('Error (m)')
    hold on
    
    figure(3)
    plot(1:25, del_MI(j,:), 'LineWidth',1.5)
    title("Change in Max Iterations")
    xlabel("Run")
    ylabel('Error (m)')
    hold on 
    
    figure(4)
    plot(1:25, del_el(j,:), 'LineWidth',1.5)
    title("Change in Damping Coefficent")
    xlabel("Run")
    ylabel('Error (m)')
    hold on
end
figure(1)
w_arr = linspace(.1,2,4);
legend(num2str(w_arr(1)), num2str(w_arr(2)), num2str(w_arr(3)), num2str(w_arr(4)))
set(gca, 'YScale', 'log')

figure(2)
al_ar= linspace(.05,.5,4);
legend(num2str(al_ar(1)), num2str(al_ar(2)), num2str(al_ar(3)), num2str(al_ar(4)))
set(gca, 'YScale', 'log')
figure(3)
MI_arr = linspace(100,1000,4);
legend(num2str(MI_arr(1)), num2str(MI_arr(2)), num2str(MI_arr(3)), num2str(MI_arr(4)))
set(gca, 'YScale', 'log')
figure(4)
el_arr = linspace(.001,.2,4);
legend(num2str(el_arr(1)), num2str(el_arr(2)), num2str(el_arr(3)), num2str(el_arr(4)))
set(gca, 'YScale', 'log')