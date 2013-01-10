
% Werte aus der Simulation
rho=800;

p1=5e5;
p3=1e5;

d12=5e-3; 
zeta12=3;
d23=7e-3; 
zeta23=1.5;


% abgeleitete Groessen
A12=pi*d12^2/4;
A23=pi*d23^2/4;

pLoss=abs(p3-p1); % bar
Q13 = sign(p1-p3)*sqrt(pLoss/(rho/2*(zeta12/A12^2+zeta23/A23^2))); % m^3/s

p2=p1-rho/2*zeta12*Q13*abs(Q13)/A12^2;

fprintf('Stationaere Loesung:\n  Q12 = Q23 = %f [l/min]\n  p2 = %f [bar]\n', Q13*6e4, p2*1e-5)
