function hi = reference(t,i,r,w)
hi = [ r*sin(w*t+(i-1)*pi/5);
       r*w*cos(w*t+(i-1)*pi/5);
       r*cos(w*t+(i-1)*pi/5);
       -w*r*cos(w*t+(i-1)*pi/5)];    
end