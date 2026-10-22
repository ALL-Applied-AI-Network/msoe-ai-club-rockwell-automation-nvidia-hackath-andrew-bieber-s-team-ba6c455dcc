"""Offline shell/restart regressions. Fake uv never contacts any remote service."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parent


class WorkflowTests(unittest.TestCase):
    def sandbox(self, tmp):
        root=Path(tmp)
        for name in ('setup_rosie.sh','request_test.sh'):
            shutil.copy2(ROOT/name,root/name)
        (root/'submission').mkdir()
        (root/'submission/placement.yaml').write_text('rollouts: []\n')
        (root/'bin').mkdir()
        uv=root/'bin/uv'
        uv.write_text('#!/usr/bin/env bash\n'
                      'printf "%s\\n" "$*" >> "$WORKFLOW_LOG"\n'
                      'if [[ -e "$WORKFLOW_FAIL" && "$1" == sync ]]; then exit 9; fi\n')
        uv.chmod(0o755)
        env={**os.environ,'PATH':str(root/'bin')+os.pathsep+os.environ['PATH'],
             'WORKFLOW_LOG':str(root/'calls.log'),'WORKFLOW_FAIL':str(root/'interrupted')}
        return root,env

    def test_request_generation_does_not_submit_or_connect(self):
        with tempfile.TemporaryDirectory() as tmp:
            root,env=self.sandbox(tmp)
            private=root/'.request.json';private.write_text('private-existing-request')
            proc=subprocess.run(['bash',str(root/'request_test.sh')],input='test@localhost\n',
                                text=True,capture_output=True,env=env,check=True)
            calls=(root/'calls.log').read_text().splitlines()
            self.assertEqual(len(calls),2)
            self.assertTrue(calls[0].startswith('run prepare_submission.py --contact Andrew Bieber, test@localhost'))
            self.assertEqual(calls[1],'run preflight.py --submission')
            self.assertIn('Nothing has been submitted',proc.stdout)
            self.assertEqual(private.read_text(),'private-existing-request')

    def test_setup_stops_at_failed_sync_and_can_restart(self):
        with tempfile.TemporaryDirectory() as tmp:
            root,env=self.sandbox(tmp)
            (root/'interrupted').write_text('simulate interrupted install')
            private=root/'.request.json';private.write_text('keep-private')
            failed=subprocess.run(['bash',str(root/'setup_rosie.sh')],env=env,capture_output=True)
            self.assertEqual(failed.returncode,9)
            self.assertEqual((root/'calls.log').read_text().splitlines(),['sync --locked'])
            (root/'interrupted').rename(root/'previous-interruption')
            subprocess.run(['bash',str(root/'setup_rosie.sh')],env=env,capture_output=True,check=True)
            calls=(root/'calls.log').read_text().splitlines()[1:]
            self.assertEqual(calls[0],'sync --locked')
            self.assertTrue(any('unittest' in c for c in calls))
            self.assertEqual(calls[-1],'run run.py --mock --trials 1 --trial-s 3')
            self.assertFalse(any('--submit' in c or '--status' in c for c in calls))
            self.assertEqual(private.read_text(),'keep-private')


if __name__=='__main__':
    unittest.main()
