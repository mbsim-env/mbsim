#!/usr/bin/python

from __future__ import print_function
import subprocess
import fileinput
import re

def run():
  WEBDIR="/var/www/html/mbsim/mergedFeeds"
  
  # Build feeds
  subprocess.check_call(["/home/mbsim/3rdparty/rsstool-1.0.1rc2-src/src/rsstool", "-r", "--rss",
    "-o="+WEBDIR+"/build.rss.xml",
    "/var/www/html/mbsim/linux64-dailydebug/report/result.rss.xml",
    "/var/www/html/mbsim/linux64-dailydebug/report/result_current/runexamples_report/result.rss.xml",
    "/var/www/html/mbsim/linux64-dailydebug/report/runexamples_valgrind_report/result.rss.xml",
    "/var/www/html/mbsim/linux64-ci/report/result.rss.xml",
    "/var/www/html/mbsim/linux64-ci/report/result_current/runexamples_report/result.rss.xml",
    "/var/www/html/mbsim/linux64-dailyrelease/report/result.rss.xml",
    "/var/www/html/mbsim/linux64-dailyrelease/report/result_current/runexamples_report/result.rss.xml",
    "/var/www/html/mbsim/linux64-dailyrelease/report_distribute/result.rss.xml",
    "/var/www/html/mbsim/win64-dailyrelease/report/result.rss.xml",
    "/var/www/html/mbsim/win64-dailyrelease/report/result_current/runexamples_report/result.rss.xml",
    "/var/www/html/mbsim/win64-dailyrelease/report_distribute/result.rss.xml"])
  # fix CEST to CET (CEST does not work on all clients)
  for line in fileinput.FileInput(WEBDIR+"/build.rss.xml", inplace=1):
    line=re.sub('<pubDate>(.*)CEST</pubDate>', r'<pubDate>\1CET</pubDate>', line)
    print(line, end="")



if __name__=="__main__":
  run()
