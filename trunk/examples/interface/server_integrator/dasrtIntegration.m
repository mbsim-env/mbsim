1+1;

% For running this file, you need the octave-sockets package!!!
% http://octave.sourceforge.net/sockets/

% loading the package
pkg load sockets

% the defined IPC messages, cp [...]/mbsim-env/kernel/mbsim/integrators/server_integrator_messages.h
IPC._SI_getTime_SI_ = '11';
IPC._SI_getzSize_SI_ = '12';
IPC._SI_getz_SI_ = '13';
IPC._SI_getzdot_SI_ = '14';
IPC._SI_getsvSize_SI_ = '15';
IPC._SI_getsv_SI_ = '16';
IPC._SI_setTime_SI_ = '17';
IPC._SI_setz_SI_ = '18';
IPC._SI_plot_SI_ = '19';
IPC._SI_shift_SI_ = '20';
IPC._SI_exitRequest_SI_ = '99';

client = socket(AF_INET, SOCK_STREAM, 0);
server_info = struct('addr', 'localhost', 'port', 4567);
rc = connect(client, server_info);

function ret=vec2str(vec)
  if (length(vec)>1)
    ret='[';
    for i=1:(length(vec)-1)
        ret=sprintf('%s%+18.16e;', ret, vec(i));
    endfor
    ret=sprintf('%s%+18.16e]', ret, vec(length(vec)));
  else
    ret=sprintf('%+18.16e', vec(1));
  endif
endfunction

function ret=myCom(client, msg)
  % Send a string from client
  rc = send(client, msg);
  if ( rc ~= length(msg) )
    fprintf('Not the full message was transmitted!')
  end
  
  % Receive message from server
  fullMsg='';
  msg_c=0;
  while (msg_c!=35) % the sign '#' is used for signalizing end-messages
    [msg_c, len_c] = recv(client, 100);
    if exist('msg_c')
      fullMsg=[fullMsg msg_c];
    else
      break;
    endif;
  endwhile

  % if a message was received, convert it to numbers
  fullMsg=fullMsg(1:(length(fullMsg)-1));
  ret=NaN;
  if (length(fullMsg)>0)
    eval(sprintf('ret=%s;', fullMsg));
  endif
endfunction

function fQuit(client, IPC)
  fprintf('Beende mbsim\n');
  myCom(client, IPC._SI_exitRequest_SI_);
  myCom(client, IPC._SI_exitRequest_SI_);
  disconnect(client)
  for i=1:2
    client = socket(AF_INET, SOCK_STREAM, 0);
    server_info = struct('addr', 'localhost', 'port', 4567);
    rc = connect(client, server_info);
    myCom(client, IPC._SI_exitRequest_SI_);
    disconnect(client)
  end
endfunction

residuum=@(x, xdot, t) fResiduum(x, xdot, t, client, IPC);
function res=fResiduum(x, xdot, t, client, IPC)
  myCom(client, sprintf('%s%s', IPC._SI_setTime_SI_, vec2str(t)));
  myCom(client, sprintf('%s%s', IPC._SI_setz_SI_, vec2str(x)));

  res=xdot-myCom(client, sprintf('%s', IPC._SI_getzdot_SI_));
endfunction

stopVector=@(x, t) fStopVector(x, t, client, IPC);
function sv=fStopVector(x, t, client, IPC)
  myCom(client, sprintf('%s%s', IPC._SI_setTime_SI_, vec2str(t)));
  myCom(client, sprintf('%s%s', IPC._SI_setz_SI_, vec2str(x)));

  sv=myCom(client, sprintf('%s', IPC._SI_getsv_SI_));
endfunction

plotFunction=@(x, t) fPlotFunction(x, t, client, IPC);
function fPlotFunction(x, t, client, IPC)
  myCom(client, sprintf('%s%s', IPC._SI_setTime_SI_, vec2str(t)));
  myCom(client, sprintf('%s%s', IPC._SI_setz_SI_, vec2str(x)));
  myCom(client, sprintf('%s', IPC._SI_plot_SI_));
endfunction

% integration and plot sizes
tEnd=1;
dtPlot=1e-2;

% options for dasrt
dasrt_options('absolute tolerance', 1e-9)
dasrt_options('relative tolerance', 1e-9)
dasrt_options('initial step size', sqrt(eps))
dasrt_options('maximum step size', 1e-2)

% get first initial conditions from system
t0=myCom(client, IPC._SI_getTime_SI_);
z0=myCom(client, IPC._SI_getz_SI_);
zdot0=myCom(client, IPC._SI_getzdot_SI_);

while (t0<tEnd)

  % define the output time
  plotTime=[t0 (dtPlot*floor((t0+dtPlot)/dtPlot)):dtPlot:tEnd];

  % do the integration until an event occurs
  [XX, XXDOT, TT_OUT, IISTAT, MMSG] = dasrt(residuum, stopVector, z0, zdot0, plotTime);

  % plot until event
  for i=1:length(TT_OUT)
    plotFunction(XX(i,:), TT_OUT(i));
  endfor
  
  % set new initial conditions
  z0=XX(length(TT_OUT), :)';
  zdot0=XXDOT(length(TT_OUT), :)';
  t0=TT_OUT(length(TT_OUT))';
  myCom(client, sprintf('%s', IPC._SI_shift_SI_));
  z0=myCom(client, IPC._SI_getz_SI_);
  zdot0=myCom(client, IPC._SI_getzdot_SI_);
endwhile


fQuit(client, IPC)
