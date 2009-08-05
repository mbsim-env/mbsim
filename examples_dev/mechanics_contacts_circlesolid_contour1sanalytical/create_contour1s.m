1;
clear;

function radius=fr(phi)
  rmin = .07;
  phiStart1=120/180*pi;
  phiMax1=160/180*pi;
  phiEnd1=210/180*pi;
  rmax1=.09;
  phiStart2=270/180*pi;
  phiMax2=300/180*pi;
  phiEnd2=340/180*pi;
  rmax2=.08;
  
  e=1e-6;
  if (phi <= phiStart1)
    radius=rmin;
  elseif (phi <= phiMax1)
    x=[phiStart1 phiStart1+e phiStart1+2*e phiMax1-2*e phiMax1-e phiMax1];
    y=[rmin rmin rmin rmax1 rmax1 rmax1];
    radius=interp1(x, y, phi, 'spline');
  elseif (phi <= phiEnd1)
    x=[phiMax1 phiMax1+e phiMax1+2*e phiEnd1-2*e phiEnd1-e phiEnd1];
    y=[rmax1 rmax1 rmax1 rmin rmin rmin];
    radius=interp1(x, y, phi, 'spline');
  elseif (phi <= phiStart2)
    radius=rmin;
  elseif (phi <= phiMax2)
    x=[phiStart2 phiStart2+e phiStart2+2*e phiMax2-2*e phiMax2-e phiMax2];
    y=[rmin rmin rmin rmax2 rmax2 rmax2];
    radius=interp1(x, y, phi, 'spline');
  elseif (phi <= phiEnd2)
    x=[phiMax2 phiMax2+e phiMax2+2*e phiEnd2-2*e phiEnd2-e phiEnd2];
    y=[rmax2 rmax2 rmax2 rmin rmin rmin];
    radius=interp1(x, y, phi, 'spline');
  else 
    radius=rmin;
  end;
endfunction;


phi=linspace(0, 2*pi, 18);
for i=1:length(phi)
  r(i)=fr(phi(i));
  rcomplex(i)=r(i)*exp(complex(0, phi(i)));
end

x=real(rcomplex);
y=imag(rcomplex);

myfile = fopen ('contour.asc', 'w');
fprintf(myfile, '%ix2\n', length(x));
fprintf(myfile, '[ %12.11e %12.11e\n', x(1), y(1));
for i=2:length(x)-1
  fprintf(myfile, '  %12.11e %12.11e\n', x(i), y(i));
end
fprintf(myfile, '  %12.11e %12.11e]', x(length(1)), y(length(1)));
fclose(myfile);
