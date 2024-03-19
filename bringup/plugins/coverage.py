import json
import os
import subprocess
import logging as log

from ..config import FUZZ_HOME, NAV2_HOME, round

class Coverage:
    def __init__(self):
        # 这个是这个fuzz的初始覆盖率基准 
        self.base_file = f'{FUZZ_HOME}/log/covr.json' 

        # Check if base summary.json exists
        self._path_exists(FUZZ_HOME + '/log/covr.json')
        #??
        self.prev_covr = self._read(self.base_file)

    def _read(self, cov_file):
        with open(cov_file, "rb") as f:
            data = json.load(f)
        covr = data.get('branch_percent', None)
        if covr is not None:
            return covr
        else:
            raise ValueError('Invalid format of covr report file')
    
    def update(self):
        # generate new covr summary file
        cmd = [
            'gcovr', '--json-summary-pretty', 
            '--gcov-executable', 'llvm-cov gcov',
            '--output', self.base_file
        ]
        result = subprocess.run(cmd, cwd=NAV2_HOME)
        if result.returncode:
            log.error(f"[Fuzz]:error calling gcovr {result}")
        else:
            log.info("[Fuzz]:gcovr successfully called")
        log.info("[Fuzz]: %d branches increase")
    
    @staticmethod
    def _path_exists(path):
        # Check if path exists, if not, create it.
        if not os.path.exists(path):
            dir_name = os.path.dirname(path)
            if not os.path.exists(dir_name):
                os.makedirs(dir_name)
