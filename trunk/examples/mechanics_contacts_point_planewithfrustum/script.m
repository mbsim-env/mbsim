% a small octave script for testing the contactconfiguration
1;
clear;




R=7;
r=6.5;
rho=.1;
h=-6; 
epsilon=1e-6;

% Statische Berechnungen aus Geometrieueberlegungen
ET = [h; r; 0];
EP = [0; R; 0];

% Mittelpunkte der Radien
tan_alpha=h/(R-r);
alpha=atan(tan_alpha);
MT= ET - sign(h)*[rho; (1/cos(alpha)-1)*rho/tan_alpha; 0];
MP= EP + sign(h)*[rho; (1/cos(alpha)-1)*rho/tan_alpha; 0];

% Bereichsgrenzen
r_Top = MT(2);
r_Plane = MP(2);
r_FrustumTop = r_Top + sign(h)*rho*sin(alpha);
r_FrustumPlane = r_Plane - sign(h)*rho*sin(alpha);

% statische Daten des Frustums
nFrustum=sign(h)*[(R-r)/h; 1; 0]; nFrustum=nFrustum/norm(nFrustum);
tFrustum=cross(nFrustum, [0; 0; 1]); tFrustum=tFrustum/norm(tFrustum);







% Daten zur Programmlaufzeit
%WrOPoint=[3; 1; 2.5];
%WrOPlane=[0; 0; 0];
%WnPlane=[0; 0; 1];       WnPlane=WnPlane/norm(WnPlane);
%WtPlane=[0; -1; 0];       WtPlane=WtPlane/norm(WtPlane);
WrOPoint=[-2.7; -7; 0];
WrOPoint=[     -0.0686592    -0.0133034     0.0142216 ]'*1e2;
WrOPlane=[0; 0; 0];
WnPlane=[0; 1; 0];       WnPlane=WnPlane/norm(WnPlane);
WtPlane=[1; 0; 0];       WtPlane=WtPlane/norm(WtPlane);

% Vektor vom Plane zum Point
WrPlanePoint = -WrOPlane + WrOPoint
  
% Abstand des Punktes von der Normalenachse
d = norm(cross(WnPlane, WrPlanePoint))

% Aufstellen des lokalen KOS
xAKW = WnPlane;
if (d<epsilon)
  yAKW=WtPlane;
else
  yAKW=cross(cross(xAKW, WrPlanePoint), xAKW);
  yAKW=yAKW/norm(yAKW);
end
zAKW=cross(xAKW, yAKW); % Vorzeichen (TESTEN!)
zAKW=zAKW/norm(zAKW);
AKW=[xAKW, yAKW, zAKW]
AKW=[xAKW, yAKW, zAKW]'
KrOPoint=AKW*WrPlanePoint


% n normale Kontaktkraftrichtung
% t tantentiale Kontaktkraftrichtung
% b binormale Kontaktkraftrichtung
% g Abstand (>0 offen, <0 geschossen)
% KrCP lokaler Vektor des potentiellen Kontaktpunktes auf Kontur
b=[0; 0; 1]; % Binormale bleibt immer identisch
if (d<r_Top) % Kontakt mit kleiner Flaeche
  sprintf('Fall 1')
  n=[1; 0; 0];
  t=[0; -1; 0];
  g=KrOPoint(1)-h;
  KrCP=[h; KrOPoint(2); 0];
elseif (d<r_FrustumTop) % Kontakt mit innerer Rundung
  sprintf('Fall 2')
  sin_alpha=(KrOPoint(2)-MT(2))/rho
  asin(sin_alpha)
  cos_alpha=cos(asin(sin_alpha))
  n=[cos_alpha; sign(h)*sin_alpha; 0]
  t=[sign(h)*sin_alpha; -cos_alpha; 0]
  KrCP=MT+sign(h)*rho*n
  KrPointCP=-KrOPoint+KrCP
  g=-sign(KrPointCP'*[1; 0; 0])*norm(KrPointCP)
elseif (d<r_FrustumPlane) % Kontakt mit seitlicher Ebene
  sprintf('Fall 3')
  n=nFrustum
  t=tFrustum
  if (h<0)
    KrExP=-EP+KrOPoint;
    KrCP=EP+(KrExP'*tFrustum)*tFrustum;
  else
    KrExP=-ET+KrOPoint;
    KrCP=ET+(KrExP'*tFrustum)*tFrustum;
  endif
  g=sign(KrExP'*nFrustum)*norm(KrCP-KrOPoint)
elseif (d<r_Plane) % Kontakt mit aeusserer Rundung
  sprintf('Fall 4')
  sin_alpha=(-KrOPoint(2)+MP(2))/rho;
  cos_alpha=cos(asin(sin_alpha));
  n=[cos_alpha; sign(h)*sin_alpha; 0];
  t=[sign(h)*sin_alpha; -cos_alpha; 0];
  KrCP=MP-sign(h)*rho*n;
  KrPointCP=-KrOPoint+KrCP;
  g=-sign(KrPointCP'*[1; 0; 0])*norm(KrPointCP);
else % Kontakt mit Ebene
  sprintf('Fall 5')
  n=[1; 0; 0];
  t=[0; -1; 0];
  g=KrOPoint(1);
  KrCP=[0; KrOPoint(2); 0];
end

AWK=AKW';
Wn=AWK*n
Wt=AWK*t
Wb=AWK*b
WrCP=WrOPlane+AWK*KrCP
