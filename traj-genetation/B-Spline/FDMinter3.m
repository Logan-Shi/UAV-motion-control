function df = FDMinter3(t,f)
df = zeros(3,length(t));
for i = 2:length(t)-1%���Ĳ�����ֵ
    df(:,i) = (f(:,i+1)-f(:,i-1))/(t(i+1)-t(i-1));
end
df(:,1) = (f(:,2)-f(:,1))/(t(2)-t(1));%��������ֵ��ֵ
df(:,end) = (f(:,end)-f(:,end-1))/(t(end)-t(end-1));%��ǰ������ֵĩֵ
end

