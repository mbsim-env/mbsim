# Dashboard

Current Daily Build Status of the MBSim-Environment

| Build Type | Variant | Failed |
|------------|---------|--------|
| linux64-dailydebug | build | [![image](https://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailydebug-build.nrFailed.svg) / ![image](https://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailydebug-build.nrAll.svg)](https://www.mbsim-env.de/mbsim/linux64-dailydebug/report/result_current/) |
| linux64-dailydebug | examples | [![image](https://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailydebug-examples.nrFailed.svg) / ![image](https://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailydebug-examples.nrAll.svg)](https://www.mbsim-env.de/mbsim/linux64-dailydebug/report/result_current/runexamples_report/result_current/) |
| linux64-dailydebug | coverage | [![image](https://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailydebug-coverage.svg)](https://www.mbsim-env.de/mbsim/linux64-dailydebug/report/result_current/runexamples_report/result_current/coverage/) |
| linux64-dailydebug | examples-valgrind | [![image](https://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailydebug-valgrind-examples.nrFailed.svg) / ![image](https://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailydebug-valgrind-examples.nrAll.svg)](https://www.mbsim-env.de/mbsim/linux64-dailydebug/report/runexamples_valgrind_report/result_current/) |
| linux64-dailydebug | coverage-valgrind | [![image](https://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailydebug-valgrind-coverage.svg)](https://www.mbsim-env.de/mbsim/linux64-dailydebug/report/runexamples_valgrind_report/result_current/coverage/) |
| linux64-dailyrelease | build | [![image](https://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailyrelease-build.nrFailed.svg) / ![image](https://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailyrelease-build.nrAll.svg)](https://www.mbsim-env.de/mbsim/linux64-dailyrelease/report/result_current/) |
| linux64-dailyrelease | examples | [![image](https://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailyrelease-examples.nrFailed.svg) / ![image](https://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailyrelease-examples.nrAll.svg)](https://www.mbsim-env.de/mbsim/linux64-dailyrelease/report/result_current/runexamples_report/result_current/) |
| win64-dailyrelease | build | [![image](https://www.mbsim-env.de/mbsim/buildsystemstate/win64-dailyrelease-build.nrFailed.svg) / ![image](https://www.mbsim-env.de/mbsim/buildsystemstate/win64-dailyrelease-build.nrAll.svg)](https://www.mbsim-env.de/mbsim/win64-dailyrelease/report/result_current/) |
| win64-dailyrelease | examples | [![image](https://www.mbsim-env.de/mbsim/buildsystemstate/win64-dailyrelease-examples.nrFailed.svg) / ![image](https://www.mbsim-env.de/mbsim/buildsystemstate/win64-dailyrelease-examples.nrAll.svg)](https://www.mbsim-env.de/mbsim/win64-dailyrelease/report/result_current/runexamples_report/result_current/) |
| - | manuals | [![image](https://www.mbsim-env.de/mbsim/buildsystemstate/build-manuals.nrFailed.svg) / ![image](https://www.mbsim-env.de/mbsim/buildsystemstate/build-manuals.nrAll.svg)](https://www.mbsim-env.de/mbsim/doc_manualsbuild.log) |



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
