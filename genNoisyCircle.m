function[xr, yr] = genNoisyCircle(H, K, R, s, N)
   xr=zeros(N,1);
   yr=zeros(N,1);
   for i=1:N
   	x=2*rand(1)*R-R;
      if rand(1) > 0.5
   		y=sqrt(R * R - x * x);
   	else
  			y=-sqrt(R * R - x * x);
   	end
   	xr(i)=x + H + rand(1) * s;
   	yr(i)=y + K + rand(1) * s;
   end
end

