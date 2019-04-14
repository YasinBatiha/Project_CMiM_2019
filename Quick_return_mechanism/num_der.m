function dx = num_der(t, x)
dx = zeros(size(x,1)-1,size(x,2));
for ii = 1:size(x,1)-1
  dx(ii,:) = (x(ii+1,:) - x(ii,:))/(t(ii+1) - t(ii));
end
end