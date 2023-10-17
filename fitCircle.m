function[h, k, r] = fitCircle(x,y)
   N=length(x);
   A=zeros(N, 3);
   b=zeros(N,1);
   for i=1:N
      A(i,1) = x(i);
      A(i,2) = y(i);
      A(i,3) = 1;
      b(i,1) = -x(i).^2-y(i).^2;
   end
   A,b
   s=A\b
   h = -0.5 * s(1);
   k = -0.5 * s(2);
   r = sqrt(h * h + k * k - s(3));
end

