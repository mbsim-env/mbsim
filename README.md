# Dashboard

Current Daily Build Status of the MBSim-Environment

| Build Type | Variant | Failed | Passed |
|------------|---------|--------|--------|
| linux64-dailydebug | build | [![image](http://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailydebug-build.nrFailed.svg)](http://www.mbsim-env.de/mbsim/linux64-dailydebug/report/result_current/) | [![image](http://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailydebug-build.nrPassed.svg)](http://www.mbsim-env.de/mbsim/linux64-dailydebug/report/result_current/) |
| linux64-dailydebug | examples | [![image](http://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailydebug-examples.nrFailed.svg)](http://www.mbsim-env.de/mbsim/linux64-dailydebug/report/result_current/runexamples_report/result_current/) | [![image](http://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailydebug-examples.nrPassed.svg)](http://www.mbsim-env.de/mbsim/linux64-dailydebug/report/result_current/runexamples_report/result_current/) |
| linux64-dailydebug | valgrind-examples | [![image](http://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailydebug-valgrind-examples.nrFailed.svg)](http://www.mbsim-env.de/mbsim/linux64-dailydebug/report/runexamples_valgrind_report/result_current/) | [![image](http://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailydebug-valgrind-examples.nrPassed.svg)](http://www.mbsim-env.de/mbsim/linux64-dailydebug/report/runexamples_valgrind_report/result_current/) |
| linux64-dailyrelease | build | [![image](http://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailyrelease-build.nrFailed.svg)](http://www.mbsim-env.de/mbsim/linux64-dailyrelease/report/result_current/) | [![image](http://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailyrelease-build.nrPassed.svg)](http://www.mbsim-env.de/mbsim/linux64-dailyrelease/report/result_current/) |
| linux64-dailyrelease | examples | [![image](http://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailyrelease-examples.nrFailed.svg)](http://www.mbsim-env.de/mbsim/linux64-dailyrelease/report/result_current/runexamples_report/result_current/) | [![image](http://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailyrelease-examples.nrPassed.svg)](http://www.mbsim-env.de/mbsim/linux64-dailyrelease/report/result_current/runexamples_report/result_current/) |
| linux64-dailyrelease | distribution | [![image](http://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailyrelease-distribution.nrFailed.svg)](http://www.mbsim-env.de/mbsim/linux64-dailyrelease/report_distribute/distribute.out) | [![image](http://www.mbsim-env.de/mbsim/buildsystemstate/linux64-dailyrelease-distribution.nrPassed.svg)](http://www.mbsim-env.de/mbsim/linux64-dailyrelease/report_distribute/distribute.out) |
| win64-dailyrelease | build | [![image](http://www.mbsim-env.de/mbsim/buildsystemstate/win64-dailyrelease-build.nrFailed.svg)](http://www.mbsim-env.de/mbsim/win64-dailyrelease/report/result_current/) | [![image](http://www.mbsim-env.de/mbsim/buildsystemstate/win64-dailyrelease-build.nrPassed.svg)](http://www.mbsim-env.de/mbsim/win64-dailyrelease/report/result_current/) |
| win64-dailyrelease | examples | [![image](http://www.mbsim-env.de/mbsim/buildsystemstate/win64-dailyrelease-examples.nrFailed.svg)](http://www.mbsim-env.de/mbsim/win64-dailyrelease/report/result_current/runexamples_report/result_current/) | [![image](http://www.mbsim-env.de/mbsim/buildsystemstate/win64-dailyrelease-examples.nrPassed.svg)](http://www.mbsim-env.de/mbsim/win64-dailyrelease/report/result_current/runexamples_report/result_current/) |
| win64-dailyrelease | distribution | [![image](http://www.mbsim-env.de/mbsim/buildsystemstate/win64-dailyrelease-distribution.nrFailed.svg)](http://www.mbsim-env.de/mbsim/win64-dailyrelease/report_distribute/distribute.out) | [![image](http://www.mbsim-env.de/mbsim/buildsystemstate/win64-dailyrelease-distribution.nrPassed.svg)](http://www.mbsim-env.de/mbsim/win64-dailyrelease/report_distribute/distribute.out) |
| - | manuals | [![image](http://www.mbsim-env.de/mbsim/buildsystemstate/build-manuals.nrFailed.svg)](http://www.mbsim-env.de/mbsim/doc_manualsbuild.log) | [![image](http://www.mbsim-env.de/mbsim/buildsystemstate/build-manuals.nrPassed.svg)](http://www.mbsim-env.de/mbsim/doc_manualsbuild.log) |



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
