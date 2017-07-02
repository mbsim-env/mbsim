include /opt/firejail-local/etc/firejail/default.profile
private
private-dev
private-tmp
caps.drop all
noroot
nogroups
nosound
mkdir /tmp/mbsimwebapp-pid
whitelist /tmp/mbsimwebapp-pid
