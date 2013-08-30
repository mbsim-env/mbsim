1+1;

% For running this file, you need the octave-sockets package!!!
% http://octave.sourceforge.net/sockets/

% loading the package
pkg load sockets

% the defined IPC messages
[~,myAddPath]=system("pkg-config --cflags mbsimInterface | sed -e 's/^.*-I//g; s/ //g'");
myAddPath=myAddPath(1:(length(myAddPath)-1));
myAddPath=sprintf('%s/mbsimInterface/', myAddPath);
fprintf('Fuege Pfad >>>%s<<< hinzu!\n', myAddPath)
addpath(myAddPath);
interfaceMessages

client = socket(AF_INET, SOCK_STREAM, 0);
server_info = struct('addr', 'localhost', 'port', 4567);
rc = connect(client, server_info);

function ret=myCom(client, msg)
  % Send a string from client
  rc = send(client, sprintf('%s\0', msg));
  if ( rc ~= length(msg) )
    fprintf('Not the full message was transmitted!')
  end
  
  % Receive message from server
  [msg_c, len_c] = recv(client, 1024*1024);

  % if a message was received, convert it to numbers
  ret=NaN;
  if (len_c>1)
    eval(sprintf('ret=%s;', char(msg_c)));
  endif
endfunction


residuum=@(x, xdot, t) fResiduum(x, xdot, t, client, IPC);
function res=fResiduum(x, xdot, t, client, IPC)
  myCom(client, sprintf('%s%s', IPC._SI_setTime_asciiString_SI_, mat2str(t)));
  myCom(client, sprintf('%s%s', IPC._SI_setStateVector_asciiString_SI_, mat2str(x)));

  res=xdot-myCom(client, sprintf('%s', IPC._SI_getTimeDerivativeOfStateVector_asciiString_SI_));
endfunction

stopVector=@(x, t) fStopVector(x, t, client, IPC);
function sv=fStopVector(x, t, client, IPC)
  myCom(client, sprintf('%s%s', IPC._SI_setTime_asciiString_SI_, mat2str(t)));
  myCom(client, sprintf('%s%s', IPC._SI_setStateVector_asciiString_SI_, mat2str(x)));

  sv=myCom(client, sprintf('%s', IPC._SI_getStopVector_asciiString_SI_));

  %myCom(client, IPC._SI_doPrintCommunication_SI_);
  %myCom(client, sprintf('%s', IPC._SI_getTime_asciiString_SI_));
  %myCom(client, IPC._SI_donotPrintCommunication_SI_);
endfunction

plotFunction=@(x, t) fPlotFunction(x, t, client, IPC);
function fPlotFunction(x, t, client, IPC)
  myCom(client, sprintf('%s%s', IPC._SI_setTime_asciiString_SI_, mat2str(t)));
  myCom(client, sprintf('%s%s', IPC._SI_setStateVector_asciiString_SI_, mat2str(x)));
  myCom(client, sprintf('%s', IPC._SI_plot_SI_));
endfunction

% integration and plot sizes
tEnd=1;
dtPlot=1e-2;

% options for dasrt
dasrt_options('absolute tolerance', 1e-4)
dasrt_options('relative tolerance', 1e-4)
dasrt_options('initial step size', sqrt(eps))
dasrt_options('maximum step size', 1e-1)

myCom(client, sprintf('%s18', IPC._SI_setAsciiPrecision_asciiString_SI_))

% get first initial conditions from system
t0=myCom(client, IPC._SI_getTime_asciiString_SI_);
z0=myCom(client, IPC._SI_getStateVector_asciiString_SI_);
zdot0=myCom(client, IPC._SI_getTimeDerivativeOfStateVector_asciiString_SI_);

myCom(client, IPC._SI_donotPrintCommunication_SI_);

while (t0<tEnd)

  % define the output time
  plotTime=[t0 (dtPlot*floor((t0+dtPlot)/dtPlot)):dtPlot:tEnd];

  % do the integration until an event occurs
  [XX, XXDOT, TT_OUT, IISTAT, MMSG] = dasrt(residuum, stopVector, z0, zdot0, plotTime);

  % plot until event
  for i=1:length(TT_OUT)
    plotFunction(XX(i,:)', TT_OUT(i));
  endfor
  
  % set new initial conditions
  z0=XX(length(TT_OUT), :)';
  zdot0=XXDOT(length(TT_OUT), :)';
  t0=TT_OUT(length(TT_OUT))';
  myCom(client, IPC._SI_shift_SI_);
  z0=myCom(client, IPC._SI_getStateVector_asciiString_SI_);
  zdot0=myCom(client, IPC._SI_getTimeDerivativeOfStateVector_asciiString_SI_);
endwhile


rc=send(client, sprintf("%s\0", IPC._SI_exitRequest_SI_))
