function f=compute_f(n,t1,t2)
    f=zeros(n+1,1);
    for i=3:length(f)
        f(i)=(t2^(i-2)-t1^(i-2))*(i+1);
    end
    f=0.02*f;
end