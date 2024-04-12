import json
import os
import subprocess

from .config import FUZZ_HOME, NAV2_HOME, round, log

class Checker:
  def __init__(self):
    self._covr_file = f'{FUZZ_HOME}/log/covr.json'
    # self._path_exists(FUZZ_HOME + '/log/covr.json')
    self._br_total = 0
    self._br_hit = 0
    self._br_new = 0
  
  @staticmethod
  def _path_exists(path):
    # Check if path exists, if not, create it.
    if not os.path.exists(path):
      dir_name = os.path.dirname(path)
      if not os.path.exists(dir_name):
        os.makedirs(dir_name)

  def work(self):
    self.have_newbug()
    self.have_newcovr()
  
  def have_newbug(self):
    pass

  def have_newcovr(self):
    # generate new covr summary file
    covr_file = f"{FUZZ_HOME}/log/covr.json" 
    cmd = [
        'gcovr', '--json-summary-pretty', 
        '--gcov-executable', 'llvm-cov gcov',
        '--output', covr_file
    ]
    result = subprocess.run(cmd, cwd=NAV2_HOME)
    if result.returncode:
      log.error(f"[Fuzz]:error calling gcovr {result}")
    else:
      log.info("[Fuzz]:gcovr successfully called")        

    try:
      with open(covr_file, "rb") as f:
        root = json.load(f)
        new_br_total = root["branch_covered"]
        new_br_hit = root["branch_total"]
        if new_br_hit > self._br_hit:
          self._br_new = new_br_hit -  self._br_hit
        self._br_total = max(self._br_total, new_br_total)
        self._br_hit = max(self._br_hit, new_br_hit)
      return True
    except Exception as e:
      log.error(f"[Fuzz][Checker]: unable to read coverage infor")
      return False