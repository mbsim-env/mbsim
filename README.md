# Dashboard

Current Daily Build Status of the MBSim-Environment

| Build Type | Variant | Failed |
|------------|---------|--------|
| linux64-dailydebug | build | [![image](https://www.mbsim-env.de/service/builds/current/linux64-dailydebug/nrFailed.svg) / ![image](https://www.mbsim-env.de/service/builds/current/linux64-dailydebug/nrAll.svg)](https://www.mbsim-env.de/builds/run/current/linux64-dailydebug/) |
| linux64-dailydebug | examples | [![image](https://www.mbsim-env.de/service/runexamples/current/linux64-dailydebug/nrFailed.svg) / ![image](https://www.mbsim-env.de/service/runexamples/current/linux64-dailydebug/nrAll.svg)](https://www.mbsim-env.de/runexamples/run/current/linux64-dailydebug/) |
| linux64-dailydebug | coverage | [![image](https://www.mbsim-env.de/service/runexamples/current/linux64-dailydebug/coverageRate.svg)](https://www.mbsim-env.de/runexamples/run/current/linux64-dailydebug/#coverage) |
| linux64-dailydebug | examples-valgrind | [![image](https://www.mbsim-env.de/service/runexamples/current/linux64-dailydebug-valgrind/nrFailed.svg) / ![image](https://www.mbsim-env.de/service/runexamples/current/linux64-dailydebug-valgrind/nrAll.svg)](https://www.mbsim-env.de/runexamples/run/current/linux64-dailydebug-valgrind/) |
| linux64-dailydebug | coverage-valgrind | [![image](https://www.mbsim-env.de/service/runexamples/current/linux64-dailydebug-valgrind/coverageRate.svg)](https://www.mbsim-env.de/runexamples/run/current/linux64-dailydebug-valgrind/#coverage) |
| linux64-dailyrelease | build | [![image](https://www.mbsim-env.de/service/builds/current/linux64-dailyrelease/nrFailed.svg) / ![image](https://www.mbsim-env.de/service/builds/current/linux64-dailyrelease/nrAll.svg)](https://www.mbsim-env.de/builds/run/current/linux64-dailyrelease/) |
| linux64-dailyrelease | examples | [![image](https://www.mbsim-env.de/service/runexamples/current/linux64-dailyrelease/nrFailed.svg) / ![image](https://www.mbsim-env.de/service/runexamples/current/linux64-dailyrelease/nrAll.svg)](https://www.mbsim-env.de/runexamples/run/current/linux64-dailyrelease/) |
| win64-dailyrelease | build | [![image](https://www.mbsim-env.de/service/builds/current/win64-dailyrelease/nrFailed.svg) / ![image](https://www.mbsim-env.de/service/builds/current/win64-dailyrelease/nrAll.svg)](https://www.mbsim-env.de/builds/run/current/win64-dailyrelease/) |
| win64-dailyrelease | examples | [![image](https://www.mbsim-env.de/service/runexamples/current/win64-dailyrelease/nrFailed.svg) / ![image](https://www.mbsim-env.de/service/runexamples/current/win64-dailyrelease/nrAll.svg)](https://www.mbsim-env.de/runexamples/run/current/win64-dailyrelease/) |
| - | manuals | [![image](https://www.mbsim-env.de/service/manuals/nrFailed.svg) / ![image](https://www.mbsim-env.de/service/manuals/nrAll.svg)](https://www.mbsim-env.de/service/home/#manuals) |



# Installation

This project provides a simulation environment for mechanical systems. It needs openmbvc++interface. After the installation proceed with the following steps.

1. Checkout from git

If not already done, checkout the git tree
`git clone https://github.com/mbsim-env/mbsim.git`

2. Run AutoTools

```
aclocal
autoheader
libtoolize -c --force
autoconf
automake -a -c --force
```

3. Proceed with configure

See file INSTALL.
