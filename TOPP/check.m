% 检测两条离散的线段相交
function intersected=check(A,F,TOL)
    intersected=0;
    As=A(:,1)';
    Fs=F(:,1)';
    for p=1:length(As)
        for q=1:length(Fs)
            if (norm(As(p)-Fs(q))<TOL) && (norm(A(p,2)-F(q,2))<TOL)
                intersected=1;
                break;
            end
        end
    end
end