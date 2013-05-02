1+1;

d=load('flowdata');

n_runs=25;
n_rows=size(d, 1);
n=n_rows/n_runs;

for i=1:n_runs

  dl=d((i-1)*n+1:(i-1)*n+n, :);

  T=dl(1,1);
  kd=dl(1,3);

  Q=dl(:,5);
  Re=dl(:,6);
  lambda=dl(:,7);
  pL=dl(:,8);

  name=sprintf('T%i_kd%5.4f_pl', T, kd)
  name=strrep(name, '.', 'k');
  f=fopen(name, 'w');
  fprintf(f, '#1: Q [l/min]\n');
  fprintf(f, '#2: Re [-]\n');
  fprintf(f, '#3: lambda [-]\n');
  fprintf(f, '#4: pV [bar]\n');
  for j=1:n
    fprintf(f, ' %f %f %f %f\n', Q(j), Re(j), lambda(j), pL(j));
 end
 fclose(f);

end
