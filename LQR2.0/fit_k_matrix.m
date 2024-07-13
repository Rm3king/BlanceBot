function poly_coeff=fit_k_matrix()
leg_value = 0.15:0.01:0.39;
num_l = length(leg_value);
K_matrixs = zeros(2,6,num_l);
%填充25组K矩阵
for i = 1:num_l
    L = leg_value(i);
    K_matrixs(:,:,i) = small_LR(L);
end
%初始化多项式系数数组
poly_coeff = zeros(2,6,4);%每个多项式6个系数
for i = 1:2
    for j = 1:6
        %提取当前位置25个-Kij
        y = squeeze(K_matrixs(i,j,:));
        %拟合多项式
        coeff = polyfit(leg_value,y,3);
        %保存系数
        poly_coeff(i,j,:) = coeff;
    end
end