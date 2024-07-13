function K = test(leglen)
poly_coeff = fit_k_matrix();
K = zeros(2,6);
for i = 1:2
    for j = 1:6
        K(i,j)=poly_coeff(i,j,1)*leglen^3+poly_coeff(i,j,2)*leglen^2+poly_coeff(i,j,3)*leglen+poly_coeff(i,j,4);
    end
end


end